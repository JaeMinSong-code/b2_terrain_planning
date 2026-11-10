#!/usr/bin/env python3
"""설정한 원본/수신 시각으로 점군·odom을 정렬해 격리된 지도에 전달한다."""

from collections import deque
import time
import signal
from pathlib import Path

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
from b2_terrain_planner.msg import LocomotionState
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener, TransformException

from elevation_common import (
    Pose, decode_xyz, interpolate, quaternion_product, rotation, unit_quaternion,
)

from elevation_self_filter import FootKinematics, JointHistory, measured_joints, keep_external_points


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
            'body_frame': '', 'body_frame_alias': '', 'use_receive_time': False,
            'max_odom_gap': 0.3, 'max_age': 1.0,
            'state_topic': '/robot/locomotion_state',
            'queue_wait': 0.35, 'reset_distance': 1.0, 'reset_angle': 1.2,
            'extrinsic_parent': '', 'extrinsic_child': '',
            'extrinsic_child_from_cloud': False,
            'extrinsic_xyz': [0.0, 0.0, 0.0], 'extrinsic_xyzw': [0.0, 0.0, 0.0, 1.0],
            'self_filter.enabled': False,
            'self_filter.nominal_joint_deg': [0.0, 44.30, -85.94],
            'self_filter.box_padding_xy': [0.0, 0.0],
            'self_filter.box_top_padding': 0.0,
            'self_filter.foot_radius': 0.032, 'self_filter.foot_padding': 0.0,
            'self_filter.joint_max_age': 0.15,
            'self_filter.joint_topic': '/robot_data',
            'self_filter.appendage_ids': ['leg_fl', 'leg_fr', 'leg_rl', 'leg_rr'],
        }
        self.cfg = {key: self.declare_parameter(key, value).value
                    for key, value in defaults.items()}
        for key in ('max_odom_gap', 'max_age', 'queue_wait', 'reset_distance', 'reset_angle'):
            if not np.isfinite(self.cfg[key]) or self.cfg[key] <= 0:
                raise ValueError(key + ' must be finite and positive')
        self.poses = deque(maxlen=2000)
        self.clouds = deque(maxlen=1)
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_pub = self.create_publisher(TFMessage, '/elevation/tf', 50)
        cloud_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.points_pub = self.create_publisher(PointCloud2, '/elevation/points', cloud_qos)
        self.status_pub = self.create_publisher(String, '/elevation/input_status', 10)
        self.clear_client = self.create_client(Trigger, '/elevation_mapping_node/clear_map')
        self.reset_pending = True
        self.clear_future = None
        self.last_status = ''
        self.last_cloud_stamp = None
        self.last_odom_stamp = None
        self.cloud_frame = None
        self.last_good = time.monotonic()
        self.create_subscription(Odometry, self.cfg['odom_topic'], self.on_odom,
                                 qos_profile_sensor_data)
        self.create_subscription(LocomotionState, self.cfg['state_topic'], self.on_state,
                                 qos_profile_sensor_data)
        self.create_subscription(PointCloud2, self.cfg['cloud_topic'], self.on_cloud, cloud_qos)
        self.create_timer(0.02, self.process)
        self.create_timer(1.0, lambda: self.status_pub.publish(String(data=self.last_status)))
        self.init_self_filter()
        self.status('waiting: pointcloud, odometry and sensor extrinsic TF')

    def init_self_filter(self):
        if not self.cfg['self_filter.enabled']:
            return
        # RobotToGui는 원본 stamp가 없으므로 실기 수신 시각 모드에서만 사용한다.
        if not self.cfg['use_receive_time']:
            raise ValueError('self filter measured joints require receive-time mode')
        from taskmsgs.msg import RobotToGui
        model = Path(get_package_share_directory('b2_terrain_planner')) / 'model/unitree_b2/b2.urdf'
        self.foot_kinematics = FootKinematics(model)
        nominal = np.tile(np.deg2rad(self.cfg['self_filter.nominal_joint_deg']), (4, 1))
        self.nominal_feet = self.foot_kinematics.feet(nominal)
        padding_xy = np.asarray(self.cfg['self_filter.box_padding_xy'], dtype=float)
        top_padding = self.cfg['self_filter.box_top_padding']
        if (padding_xy.shape != (2,) or not np.isfinite(padding_xy).all()
                or np.any(padding_xy < 0) or not np.isfinite(top_padding)
                or top_padding < 0):
            raise ValueError('invalid self filter box padding')
        self.joint_history = JointHistory()
        self.foot_radius = (self.cfg['self_filter.foot_radius']
                            + self.cfg['self_filter.foot_padding'])
        if (self.cfg['self_filter.foot_padding'] < 0 or self.foot_radius <= 0
                or self.cfg['self_filter.joint_max_age'] <= 0):
            raise ValueError('invalid self filter radius or joint age')
        self.filter_markers = self.create_publisher(MarkerArray, '/elevation/self_filter', 1)
        self.filter_status = self.create_publisher(String, '/elevation/self_filter_status', 1)
        self.create_subscription(RobotToGui, self.cfg['self_filter.joint_topic'],
                                 self.on_filter_joints, qos_profile_sensor_data)

    def on_filter_joints(self, msg):
        try:
            joints = measured_joints(msg, self.cfg['self_filter.appendage_ids'])
            self.joint_history.append(self.get_clock().now().nanoseconds * 1e-9, joints)
        except ValueError as exc:
            self.get_logger().warning('self filter joints: ' + str(exc), throttle_duration_sec=5.)

    def filter_self_points(self, xyz, sensor_p, sensor_q, stamp):
        lower = self.nominal_feet.min(axis=0)
        upper = self.nominal_feet.max(axis=0)
        # hip 바깥과 상부 장착물까지 확장하되 발밑 지면 경계는 유지한다.
        lower[:2] -= self.cfg['self_filter.box_padding_xy']
        upper[:2] += self.cfg['self_filter.box_padding_xy']
        upper[2] = sensor_p[2] + self.cfg['self_filter.box_top_padding']
        feet = np.empty((0, 3))
        joints = self.joint_history.at(seconds(stamp), self.cfg['self_filter.joint_max_age'])
        if joints is not None:
            feet = self.foot_kinematics.feet(joints)
        points_body = xyz @ rotation(sensor_q).T + sensor_p
        keep = keep_external_points(points_body, lower, upper, feet, self.foot_radius)
        self.publish_filter_markers(lower, upper, feet, stamp)
        self.filter_status.publish(String(data=(
            f'removed={np.count_nonzero(~keep)}/{len(xyz)}; measured_feet={len(feet)}/4')))
        return xyz[keep]

    def publish_filter_markers(self, lower, upper, feet, stamp):
        markers = []
        for index in range(5):
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = 'elevation_base'
            marker.ns = 'self_filter'
            marker.id = index
            marker.pose.orientation.w = 1.
            marker.lifetime = Duration(seconds=0.5).to_msg()
            if index == 0:
                marker.type = Marker.LINE_LIST
                marker.scale.x = .008
                marker.color.r, marker.color.g, marker.color.a = 1., .65, .9
                corners = np.array([[x, y, z] for x in (lower[0], upper[0])
                                    for y in (lower[1], upper[1])
                                    for z in (lower[2], upper[2])])
                for a in range(8):
                    for bit in (1, 2, 4):
                        b = a ^ bit
                        if a < b:
                            for point in (corners[a], corners[b]):
                                marker.points.append(Point(x=float(point[0]),
                                                           y=float(point[1]), z=float(point[2])))
            elif index <= len(feet):
                marker.type = Marker.SPHERE
                point = feet[index - 1]
                marker.pose.position = Point(x=float(point[0]), y=float(point[1]), z=float(point[2]))
                marker.scale.x = marker.scale.y = marker.scale.z = 2 * self.foot_radius
                marker.color.r, marker.color.b, marker.color.a = 1., .5, .45
            else:
                marker.action = Marker.DELETE
            markers.append(marker)
        self.filter_markers.publish(MarkerArray(markers=markers))

    def status(self, text):
        if text != self.last_status:
            self.get_logger().info(text)
            self.status_pub.publish(String(data=text))
            self.last_status = text

    def reset(self, reason):
        self.poses.clear()
        self.clouds.clear()
        self.last_cloud_stamp = None
        self.last_odom_stamp = None
        self.reset_pending = True
        if self.cfg.get('self_filter.enabled', False):
            self.joint_history.samples.clear()
        self.status('reset: ' + reason)

    def on_odom(self, msg):
        try:
            source_stamp = seconds(msg.header.stamp)
            if not np.isfinite(source_stamp) or source_stamp <= 0:
                raise ValueError("invalid odometry pose/frame/stamp")
            stamp = source_stamp
            if self.cfg['use_receive_time']:
                stamp = self.get_clock().now().nanoseconds * 1e-9
            pose = Pose(stamp, msg.header.frame_id, msg.child_frame_id,
                        vector(msg.pose.pose.position), quaternion(msg.pose.pose.orientation))
            if self.poses:
                previous = self.poses[-1]
                if source_stamp == self.last_odom_stamp or pose.stamp == previous.stamp:
                    return
                angle = 2 * np.arccos(np.clip(abs(pose.quaternion @ previous.quaternion), 0, 1))
                if (source_stamp < self.last_odom_stamp or pose.stamp < previous.stamp
                        or (pose.parent, pose.child) != (previous.parent, previous.child)
                        or np.linalg.norm(pose.position - previous.position)
                        > self.cfg['reset_distance'] or angle > self.cfg['reset_angle']):
                    self.reset('odometry discontinuity/frame change')
            self.poses.append(pose)
            self.last_odom_stamp = source_stamp
            if self.clouds:
                self.process()
        except ValueError as exc:
            self.status('invalid odometry: ' + str(exc))

    def on_state(self, msg):
        """지도 odom과 같은 좌표의 발 위치를 초기화용 격리 TF로 전달한다."""
        if not self.poses or msg.header.frame_id != self.poses[-1].parent:
            return
        age = self.get_clock().now().nanoseconds * 1e-9 - seconds(msg.header.stamp)
        if not 0 <= age <= self.cfg['max_age']:
            return
        feet = [vector(foot) for foot in msg.foot_position]
        if not all(np.isfinite(foot).all() for foot in feet):
            return
        frames = ('FL_foot', 'FR_foot', 'RL_foot', 'RR_foot')
        identity = np.array([0.0, 0.0, 0.0, 1.0])
        self.tf_pub.publish(TFMessage(transforms=[
            transform(msg.header.stamp, 'elevation_odom', frame, foot, identity)
            for frame, foot in zip(frames, feet)
        ]))

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
        # 원본은 보존하고 수신 시각 모드의 정렬·출력에 쓸 로컬 시각을 기록한다.
        self.clouds.append((time.monotonic(), self.get_clock().now().to_msg(), msg))
        self.process()

    def lookup(self, parent, child, stamp, sensor=False):
        if parent == child:
            return np.zeros(3), np.array([0., 0., 0., 1.])
        try:
            result = self.tf_buffer.lookup_transform(parent, child, Time.from_msg(stamp))
            position = vector(result.transform.translation)
            attitude = unit_quaternion(quaternion(result.transform.rotation))
        except TransformException:
            # 설정에서 동일 몸통으로 지정한 프레임에만 identity fallback을 적용한다.
            if (not sensor and self.cfg['body_frame_alias']
                    and parent == self.cfg['body_frame_alias']
                    and child == self.cfg['body_frame']):
                return np.zeros(3), np.array([0., 0., 0., 1.])
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
                    if not self.reset_pending:
                        self.status('waiting: pointcloud, odometry and sensor extrinsic TF')
                except Exception as exc:
                    self.status('map reset failed: ' + str(exc))
                self.clear_future = None
            return
        if not self.clouds:
            if time.monotonic() - self.last_good > self.cfg['max_age']:
                self.status('stale: no usable cloud/odometry')
            return
        received, received_stamp, msg = self.clouds[0]
        stamp = seconds(msg.header.stamp)
        output_stamp = msg.header.stamp
        if self.cfg['use_receive_time']:
            if time.monotonic() - received > self.cfg['max_age']:
                self.clouds.popleft()
                self.status('dropped: cloud expired since reception')
                return
            output_stamp = received_stamp
            stamp = seconds(received_stamp)
        else:
            now = self.get_clock().now().nanoseconds * 1e-9
            if now - stamp > self.cfg['max_age'] or stamp - now > 0.1:
                self.clouds.popleft()
                self.status('dropped: stale/future sensor stamp; check clock synchronization')
                return
        try:
            pose = interpolate(self.poses, stamp, self.cfg['max_odom_gap'])
            body = self.cfg['body_frame'] or pose.child
            body_p, body_q = self.lookup(pose.child, body, output_stamp)
            sensor_p, sensor_q = self.lookup(body, msg.header.frame_id, output_stamp,
                                            sensor=True)
            map_body_p = pose.position + rotation(pose.quaternion) @ body_p
            map_body_q = quaternion_product(pose.quaternion, body_q)
            xyz = decode_xyz(msg)
            if not len(xyz):
                raise ValueError('empty finite pointcloud')
            self.tf_pub.publish(TFMessage(transforms=[
                transform(output_stamp, 'elevation_odom', 'elevation_base',
                          map_body_p, map_body_q),
                transform(output_stamp, 'elevation_base', 'elevation_lidar',
                          sensor_p, sensor_q),
            ]))
            if self.cfg.get('self_filter.enabled', False):
                xyz = self.filter_self_points(xyz, sensor_p, sensor_q, output_stamp)
                if not len(xyz):
                    self.clouds.popleft()
                    self.last_good = time.monotonic()
                    self.status('ready: self filter removed all points in cloud')
                    return
            output = PointCloud2()
            # 보간된 같은 자세와 점군을 동일 시각으로 전달한다.
            output.header.stamp = output_stamp
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
            elif time.monotonic() - self.last_good > self.cfg['max_age']:
                # 최신 점군이 계속 교체되어도 지속적인 정렬 실패를 숨기지 않는다.
                self.status('waiting: ' + str(exc))


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
