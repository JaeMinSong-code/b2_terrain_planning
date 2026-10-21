#!/usr/bin/env python3
"""격리 TF와 정확한 timestamp 조회를 사용하는 기존 CuPy 맵 실행기."""
from collections import deque
import time
import signal
import math
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from tf2_ros import TransformException
from elevation_mapping_cupy.elevation_mapping_node import ElevationMappingNode
from geometry_msgs.msg import PointStamped
from elevation_common import decode_xyz, rotation, flat_ground_height


class StrictElevationMappingNode(ElevationMappingNode):
    def __init__(self):
        self.pending_clouds = deque(maxlen=5)
        self.last_accepted = 0.0
        super().__init__()
        self.flat_plane_pub = self.create_publisher(PointStamped, '/elevation/observed_plane', 10)
        self.input_timer = self.create_timer(0.01, self.flush_inputs)

    def pointcloud_callback(self, msg, sub_key):
        self.pending_clouds.append((time.monotonic(), msg, sub_key))

    def flush_inputs(self):
        if not self.pending_clouds:
            return
        received, msg, key = self.pending_clouds[0]
        if time.monotonic() - received > 0.3:
            self.pending_clouds.popleft()
            return
        for frame in (msg.header.frame_id, self.base_frame):
            if self.safe_lookup_transform(self.map_frame, frame, msg.header.stamp) is None:
                return
        self.pending_clouds.popleft()
        self._last_t = msg.header.stamp
        self.pose_update()
        super().pointcloud_callback(msg, key)
        if self.flat_plane_pub.get_subscription_count() > 0:
            self.publish_observed_plane(msg)
        self.last_accepted = time.monotonic()

    def publish_observed_plane(self, msg):
        """누적 fusion과 독립적으로 최신 점군/TF의 평면을 같은 source stamp로 보낸다."""
        sensor = self.safe_lookup_transform(self.map_frame, msg.header.frame_id, msg.header.stamp)
        base = self.safe_lookup_transform(self.map_frame, self.base_frame, msg.header.stamp)
        if sensor is None or base is None:
            return
        try:
            t, q = sensor.transform.translation, sensor.transform.rotation
            world = decode_xyz(msg) @ rotation([q.x, q.y, q.z, q.w]).T + np.array([t.x, t.y, t.z])
            b = base.transform.translation
            height = flat_ground_height(world, [b.x, b.y, b.z], self.param.map_length/2)
            output = PointStamped()
            output.header.stamp = msg.header.stamp
            output.header.frame_id = self.map_frame
            output.point.z = height
            self.flat_plane_pub.publish(output)
        except ValueError as error:
            self.get_logger().warning(f'Flat plane observation unavailable: {error}',
                                      throttle_duration_sec=2.0)

    def publish_map(self, key):
        if time.monotonic() - self.last_accepted <= 1.0:
            super().publish_map(key)

    def publish_policy_crop(self):
        if (time.monotonic() - self.last_accepted > 1.0
                or self._map_t is None or self._map_q is None):
            return
        q = self._map_q
        yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
        # CuPy 내부 elevation은 center.z에 상대적이다. 절대 base.z를 또 빼지 않는다.
        relative_base_z = float(self._map_t.z) - float(self._map.center[2])
        _, mask, filled = self._map.get_policy_crop(
            float(self._map_t.x), float(self._map_t.y), relative_base_z, yaw,
            self.param.policy_crop_length_x, self.param.policy_crop_length_y,
            self.param.policy_crop_resolution)
        self._policy_heightmap_pub.publish(self._pack_2d_f32(filled))
        self._policy_mask_pub.publish(self._pack_2d_f32(mask))

    def safe_lookup_transform(self, target_frame, source_frame, stamp):
        try:
            return self._tf_buffer.lookup_transform(
                target_frame, source_frame, stamp, timeout=Duration(seconds=0.0))
        except TransformException:
            return None

    def handle_clear_map(self, request, response):
        self.last_accepted = 0.0
        self.pending_clouds.clear()
        response = super().handle_clear_map(request, response)
        self._map_t = None
        self._map_q = None
        self._last_t = None
        self._tf_buffer.clear()
        return response


def main():
    rclpy.init()
    node = StrictElevationMappingNode()
    # TF와 점군의 별도 DDS 전달 순서를 수용한다.
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
