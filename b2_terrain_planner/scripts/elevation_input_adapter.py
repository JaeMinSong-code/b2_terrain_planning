#!/usr/bin/env python3
"""B2/시뮬 점군과 odometry를 격리된 elevation 좌표계로 연결한다."""

from collections import deque
import time
import signal

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener, TransformException

from elevation_common import (
    Pose, decode_xyz, interpolate, quaternion_product, rotation, unit_quaternion,
)


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def vector(v):
    return np.array([v.x, v.y, v.z])


def quaternion(q):
    return np.array([q.x, q.y, q.z, q.w])


def transform(stamp, parent, child, position, attitude):
    msg = TransformStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = parent
    msg.child_frame_id = child
    msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = (
        float(x) for x in position)
    (msg.transform.rotation.x, msg.transform.rotation.y,
     msg.transform.rotation.z, msg.transform.rotation.w) = (float(x) for x in attitude)
    return msg


class ElevationInputAdapter(Node):
    def __init__(self):
        super().__init__('elevation_input_adapter')
        defaults = {
            'cloud_topic': '/rslidar_points', 'odom_topic': '/dog_odom',
            'body_frame': '', 'max_odom_gap': 0.3, 'max_age': 1.0,
            'queue_wait': 0.35, 'reset_distance': 1.0, 'reset_angle': 1.2,
            'extrinsic_parent': '', 'extrinsic_child': '',
            'extrinsic_child_from_cloud': False,
            'extrinsic_xyz': [0.0, 0.0, 0.0], 'extrinsic_xyzw': [0.0, 0.0, 0.0, 1.0],
        }
        self.cfg = {key: self.declare_parameter(key, value).value
                    for key, value in defaults.items()}
        for key in ('max_odom_gap', 'max_age', 'queue_wait', 'reset_distance', 'reset_angle'):
            if not np.isfinite(self.cfg[key]) or self.cfg[key] <= 0:
                raise ValueError(key + ' must be finite and positive')
        self.poses = deque(maxlen=2000)
        self.clouds = deque(maxlen=8)
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_pub = self.create_publisher(TFMessage, '/elevation/tf', 50)
        self.points_pub = self.create_publisher(PointCloud2, '/elevation/points',
                                                qos_profile_sensor_data)
        self.status_pub = self.create_publisher(String, '/elevation/input_status', 10)
        self.clear_client = self.create_client(Trigger, '/elevation_mapping_node/clear_map')
        self.reset_pending = True
        self.clear_future = None
        self.last_status = ''
        self.last_cloud_stamp = None
        self.cloud_frame = None
        self.last_good = time.monotonic()
        self.create_subscription(Odometry, self.cfg['odom_topic'], self.on_odom,
                                 qos_profile_sensor_data)
        self.create_subscription(PointCloud2, self.cfg['cloud_topic'], self.on_cloud,
                                 qos_profile_sensor_data)
        self.create_timer(0.02, self.process)
        self.create_timer(1.0, lambda: self.status_pub.publish(String(data=self.last_status)))
        self.status('waiting: pointcloud, odometry and sensor extrinsic TF')

    def status(self, text):
        if text != self.last_status:
            self.get_logger().info(text)
            self.status_pub.publish(String(data=text))
            self.last_status = text

    def reset(self, reason):
        self.poses.clear()
        self.clouds.clear()
        self.last_cloud_stamp = None
        self.reset_pending = True
        self.status('reset: ' + reason)

    def on_odom(self, msg):
        try:
            pose = Pose(seconds(msg.header.stamp), msg.header.frame_id, msg.child_frame_id,
                        vector(msg.pose.pose.position), quaternion(msg.pose.pose.orientation))
            if self.poses:
                previous = self.poses[-1]
                if pose.stamp == previous.stamp:
                    return
                angle = 2 * np.arccos(np.clip(abs(pose.quaternion @ previous.quaternion), 0, 1))
                if (pose.stamp < previous.stamp
                        or (pose.parent, pose.child) != (previous.parent, previous.child)
                        or np.linalg.norm(pose.position - previous.position)
                        > self.cfg['reset_distance'] or angle > self.cfg['reset_angle']):
                    self.reset('odometry discontinuity/frame change')
            self.poses.append(pose)
        except ValueError as exc:
            self.status('invalid odometry: ' + str(exc))

    def on_cloud(self, msg):
        stamp = seconds(msg.header.stamp)
        if not msg.header.frame_id or stamp <= 0:
            self.status('invalid cloud: empty frame or zero stamp')
            return
        if self.cloud_frame is not None and self.cloud_frame != msg.header.frame_id:
            self.reset('pointcloud frame changed')
        self.cloud_frame = msg.header.frame_id
        if self.last_cloud_stamp is not None and stamp < self.last_cloud_stamp:
            self.reset('pointcloud clock moved backwards')
        if stamp == self.last_cloud_stamp:
            return
        self.last_cloud_stamp = stamp
        self.clouds.append((time.monotonic(), msg))

    def lookup(self, parent, child, stamp, sensor=False):
        if parent == child:
            return np.zeros(3), np.array([0., 0., 0., 1.])
        try:
            result = self.tf_buffer.lookup_transform(parent, child, Time.from_msg(stamp))
            position = vector(result.transform.translation)
            attitude = unit_quaternion(quaternion(result.transform.rotation))
        except TransformException:
            child_matches = child == self.cfg['extrinsic_child']
            if sensor and self.cfg['extrinsic_child_from_cloud']:
                child_matches = True
            if not self.cfg['extrinsic_parent'] or not (
                    parent == self.cfg['extrinsic_parent'] and child_matches):
                raise
            position = np.array(self.cfg['extrinsic_xyz'])
            attitude = unit_quaternion(self.cfg['extrinsic_xyzw'])
        if position.shape != (3,) or not np.isfinite(position).all() or attitude.shape != (4,):
            raise ValueError('invalid extrinsic transform')
        return position, attitude

    def process(self):
        if self.reset_pending:
            if self.clear_future is None:
                if self.clear_client.service_is_ready():
                    self.clear_future = self.clear_client.call_async(Trigger.Request())
                else:
                    self.status('waiting: /elevation_mapping_node/clear_map service')
            elif self.clear_future.done():
                try:
                    self.reset_pending = not self.clear_future.result().success
                except Exception as exc:
                    self.status('map reset failed: ' + str(exc))
                self.clear_future = None
            return
        if not self.clouds:
            if time.monotonic() - self.last_good > self.cfg['max_age']:
                self.status('stale: no usable cloud/odometry')
            return
        received, msg = self.clouds[0]
        stamp = seconds(msg.header.stamp)
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - stamp > self.cfg['max_age'] or stamp - now > 0.1:
            self.clouds.popleft()
            self.status('dropped: stale/future sensor stamp; check clock synchronization')
            return
        try:
            pose = interpolate(self.poses, stamp, self.cfg['max_odom_gap'])
            body = self.cfg['body_frame'] or pose.child
            body_p, body_q = self.lookup(pose.child, body, msg.header.stamp)
            sensor_p, sensor_q = self.lookup(body, msg.header.frame_id, msg.header.stamp,
                                            sensor=True)
            map_body_p = pose.position + rotation(pose.quaternion) @ body_p
            map_body_q = quaternion_product(pose.quaternion, body_q)
            xyz = decode_xyz(msg)
            if not len(xyz):
                raise ValueError('empty finite pointcloud')
            self.tf_pub.publish(TFMessage(transforms=[
                transform(msg.header.stamp, 'elevation_odom', 'elevation_base',
                          map_body_p, map_body_q),
                transform(msg.header.stamp, 'elevation_base', 'elevation_lidar',
                          sensor_p, sensor_q),
            ]))
            output = PointCloud2()
            output.header.stamp = msg.header.stamp
            # 센서 좌표를 유지하고 동등한 변환을 전용 TF에 발행한다.
            output.header.frame_id = 'elevation_lidar'
            output.height, output.width = 1, len(xyz)
            output.fields = [PointField(name=n, offset=i*4, datatype=7, count=1)
                             for i, n in enumerate(('x', 'y', 'z'))]
            output.point_step, output.row_step = 12, len(xyz)*12
            output.is_dense = True
            output.data = xyz.astype('<f4').tobytes()
            self.points_pub.publish(output)
            self.clouds.popleft()
            self.last_good = time.monotonic()
            self.status('ready: synchronized cloud + odometry + extrinsics')
        except (ValueError, TransformException) as exc:
            if time.monotonic() - received > self.cfg['queue_wait']:
                self.clouds.popleft()
                self.status('dropped: ' + str(exc))


def main():
    rclpy.init()
    node = ElevationInputAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
