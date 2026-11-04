#!/usr/bin/env python3
"""격리 TF와 정확한 timestamp 조회를 사용하는 기존 CuPy 맵 실행기."""
from collections import deque
import time
import signal
import math
import numpy as np
import cupy as cp
from scipy.spatial import QhullError
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from tf2_ros import TransformException
from elevation_mapping_cupy.elevation_mapping_node import ElevationMappingNode
from geometry_msgs.msg import PointStamped
from elevation_common import decode_xyz, rotation, flat_ground_height


class StrictElevationMappingNode(ElevationMappingNode):
    def __init__(self):
        self.pending_clouds = deque(maxlen=1)
        self.last_accepted = 0.0
        super().__init__()
        self.initializer_pending = self.use_initializer_at_start
        self.last_initializer_attempt = -math.inf
        self.initialize_patch_length = float(self._b2_param('initialize_patch_length', 1.2))
        self.initialize_patch_width = float(self._b2_param('initialize_patch_width', 0.8))
        self.flat_plane_pub = self.create_publisher(PointStamped, '/elevation/observed_plane', 10)
        self.input_timer = self.create_timer(0.01, self.flush_inputs)

    def pointcloud_callback(self, msg, sub_key):
        self.pending_clouds.append((time.monotonic(), msg, sub_key))
        self.flush_inputs()

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
        self.try_initialize_from_ground(msg)
        super().pointcloud_callback(msg, key)
        if self.flat_plane_pub.get_subscription_count() > 0:
            self.publish_observed_plane(msg)
        self.last_accepted = time.monotonic()

    def try_initialize_from_ground(self, msg):
        """평지 출발 시 관측 바닥 높이로 몸통 아래 미관측 사각형만 한 번 채운다."""
        if not self.initializer_pending:
            return
        wall_now = time.monotonic()
        if wall_now - self.last_initializer_attempt < 1.0:
            return
        self.last_initializer_attempt = wall_now
        sensor = self.safe_lookup_transform(self.map_frame, msg.header.frame_id, msg.header.stamp)
        base = self.safe_lookup_transform(self.map_frame, self.base_frame, msg.header.stamp)
        if sensor is None or base is None:
            return
        try:
            t, q = sensor.transform.translation, sensor.transform.rotation
            world = decode_xyz(msg) @ rotation([q.x, q.y, q.z, q.w]).T + np.array([t.x, t.y, t.z])
            b, q = base.transform.translation, base.transform.rotation
            length, width = self.initialize_patch_length, self.initialize_patch_width
            # 발 링크 중심 대신 같은 시각·프레임의 실제 LiDAR 바닥을 기준으로 쓴다.
            height = flat_ground_height(world, [b.x, b.y, b.z], max(length, width))
            yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
            c, s = math.cos(yaw), math.sin(yaw)
            corners = np.array([[-length/2, -width/2], [-length/2, width/2],
                                [length/2, -width/2], [length/2, width/2]])
            xy = corners @ np.array([[c, s], [-s, c]]) + [b.x, b.y]
            with self._map.map_lock:
                seeds = cp.asarray(np.column_stack((xy, np.full(4, height))),
                                   dtype=self._map.data_type)
                indices_xy = ((seeds[:, :2] - self._map.center[:2]) / self._map.resolution
                              + self._map.cell_n / 2)
                # native 초기화기는 월드 X/Y가 아닌 배열 행(Y)/열(X) 순서를 받는다.
                seeds[:, :2] = indices_xy[:, [1, 0]]
                seeds[:, 2] -= self._map.center[2]
                initialized = cp.zeros_like(self._map.elevation_map)
                # native linear 초기화기의 볼록껍질을 로봇 크기 사각형으로 제한한다.
                # initialize_map()은 기존 지도도 clear하므로 별도 버퍼만 사용한다.
                self._map.map_initializer(initialized, seeds, 'linear')
                missing = (self._map.elevation_map[2] <= 0.5) & (initialized[2] > 0.5)
                count = int(cp.count_nonzero(missing).item())
                self._map.elevation_map[:, missing] = initialized[:, missing]
                self._map.update_upper_bound_with_valid_elevation()
                self._map.plugin_manager.reset_layers()
            self.initializer_pending = False
            self.get_logger().info(
                f'Ground initializer complete: height={height:.4f} m, '
                f'patch={length:.2f}x{width:.2f} m, filled={count} cells')
        except (ValueError, AssertionError, QhullError) as error:
            self.get_logger().warning(f'Ground initializer retry: {error}', throttle_duration_sec=5.0)

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

    def _publish_updated_maps(self):
        # 첫 융합 결과도 기존 publish_map의 freshness 조건을 만족하게 한다.
        self.last_accepted = time.monotonic()
        super()._publish_updated_maps()

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
        if response.success:
            self.initializer_pending = self.use_initializer_at_start
            self.last_initializer_attempt = -math.inf
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
