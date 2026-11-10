#!/usr/bin/env python3
"""격리 TF와 정확한 timestamp 조회를 사용하는 CuPy 맵 실행기.

공식 elevation_mapping_cupy는 원본 그대로 사용하고, B2에 필요한 맵 노드 동작 변경은
모두 이 StrictElevationMappingNode 서브클래스와 B2ElevationMap(b2_elevation_map)에 담는다.
- 맵 파라미터(protection zone / max-fusion / policy crop)는 _set_b2_extension_params가
  upstream Parameter를 수정하지 않고 self.param에 동적 주입한다.
- 맵 로직은 initialize_elevation_mapping에서 B2ElevationMap으로 주입한다.
- 발행(격자 중심 pose, raw 레이어 일괄 복사), 즉시 발행(publish_on_update), 정책 크롭,
  QoS는 아래 override에서 처리한다.
"""
from collections import deque
from functools import partial
import time
import signal
import math
import numpy as np
import cupy as cp
from scipy.spatial import QhullError
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile
from tf2_ros import TransformException
from elevation_mapping_cupy.elevation_mapping_node import ElevationMappingNode
from grid_map_msgs.msg import GridMap
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayLayout as MAL
from std_msgs.msg import MultiArrayDimension as MAD
from b2_elevation_map import B2ElevationMap
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

    # --- B2 확장 파라미터 주입 (upstream Parameter/노드 수정 없이) -------------------
    def _b2_param(self, name, default):
        """YAML에 있으면 그 값을, 없으면 default로 선언 후 읽는다(vanilla 호환)."""
        if not self.has_parameter(name):
            self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def _set_b2_extension_params(self):
        """handoff 3수정용 파라미터(protection zone / max-fusion / policy crop)."""
        p = self.param
        p.enable_protection_zone = bool(self._b2_param('enable_protection_zone', False))
        p.protection_zone_x = float(self._b2_param('protection_zone_x', 0.5))
        p.protection_zone_y = float(self._b2_param('protection_zone_y', 0.3))
        p.enable_max_fusion = bool(self._b2_param('enable_max_fusion', False))
        p.max_fusion_edge_thresh = float(self._b2_param('max_fusion_edge_thresh', 0.03))
        p.enable_policy_crop = bool(self._b2_param('enable_policy_crop', False))
        p.policy_crop_length_x = float(self._b2_param('policy_crop_length_x', 1.6))
        p.policy_crop_length_y = float(self._b2_param('policy_crop_length_y', 1.0))
        p.policy_crop_resolution = float(self._b2_param('policy_crop_resolution', 0.05))
        p.policy_crop_fps = float(self._b2_param('policy_crop_fps', 50.0))

    def set_param_values_from_ros(self):
        # vanilla 파라미터 로드 뒤 B2 확장 파라미터를 주입한다.
        super().set_param_values_from_ros()
        self._set_b2_extension_params()

    def initialize_elevation_mapping(self):
        # vanilla 초기화와 동일하되 맵만 B2ElevationMap으로 주입한다.
        self.param.update()
        self._pointcloud_process_counter = 0
        self._image_process_counter = 0
        self._map = B2ElevationMap(self.param)
        self._map_data = np.zeros(
            (self._map.cell_n - 2, self._map.cell_n - 2), dtype=np.float32
        )
        self.get_logger().info(
            f"Initialized map with length: {self._map.map_length}, "
            f"resolution: {self._map.resolution}, cells: {self._map.cell_n}")
        self._map_q = None
        self._map_t = None

    def register_publishers(self):
        # QoS(depth=1) + publish_on_update 즉시 발행 + 레이어 버퍼 재사용.
        self._publishers_dict = {}
        self._publishers_timers = []
        self._map_layer_buffers = {}

        for pub_key, pub_config in self.my_publishers.items():
            topic_name = f"/{self.get_name()}/{pub_key}"
            publisher = self.create_publisher(GridMap, topic_name, QoSProfile(depth=1))
            self._publishers_dict[pub_key] = publisher

            if pub_config.get("publish_on_update", False):
                continue
            fps = pub_config.get("fps", 1.0)
            timer = self.create_timer(
                1.0 / fps,
                partial(self.publish_map, key=pub_key)
            )
            self._publishers_timers.append(timer)

    def register_timers(self):
        super().register_timers()
        # 수정3: 정책 입력 크롭 (H_filled / M) 발행 타이머.
        self.timer_policy_crop = None
        if self.param.enable_policy_crop:
            self._policy_heightmap_pub = self.create_publisher(
                Float32MultiArray, f"/{self.get_name()}/policy_heightmap", 1)
            self._policy_mask_pub = self.create_publisher(
                Float32MultiArray, f"/{self.get_name()}/policy_mask", 1)
            self.timer_policy_crop = self.create_timer(
                1.0 / max(1.0, self.param.policy_crop_fps),
                self.publish_policy_crop
            )
            self.get_logger().info(
                "[policy_crop] enabled: %.2fx%.2f m @ %.3f m -> %dx%d, %.1f Hz"
                % (self.param.policy_crop_length_x, self.param.policy_crop_length_y,
                   self.param.policy_crop_resolution,
                   int(round(self.param.policy_crop_length_x / self.param.policy_crop_resolution)),
                   int(round(self.param.policy_crop_length_y / self.param.policy_crop_resolution)),
                   self.param.policy_crop_fps))

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
        # T3(지연 계측): 스캔 수신→맵 반영까지 처리 시간. 크롭은 policy_crop_fps
        # 타이머로 발행되므로 scan→crop 최대 지연 ≈ 처리시간 + 크롭 주기.
        t0 = time.perf_counter()
        super().pointcloud_callback(msg, key)
        self._publish_updated_maps()
        if self.param.enable_policy_crop and (self._pointcloud_process_counter % 20 == 0):
            cp.cuda.get_current_stream().synchronize()
            proc_ms = (time.perf_counter() - t0) * 1000.0
            crop_ms = 1000.0 / max(1.0, self.param.policy_crop_fps)
            self.get_logger().info(
                "[T3] scan processing %.2f ms; scan->crop <= %.1f ms (crop period %.0f ms)"
                % (proc_ms, proc_ms + crop_ms, crop_ms))
        if self.flat_plane_pub.get_subscription_count() > 0:
            self.publish_observed_plane(msg)
        self.last_accepted = time.monotonic()

    def image_callback(self, *args, **kwargs):
        super().image_callback(*args, **kwargs)
        self._publish_updated_maps()

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
        """융합 완료 직후 선택한 출력을 발행한다(publish_on_update). 발행 타이머를 기다리지 않는다.

        첫 융합 결과도 publish_map의 freshness 조건을 만족하게 last_accepted를 갱신한다.
        """
        self.last_accepted = time.monotonic()
        for key, config in self.my_publishers.items():
            if config.get("publish_on_update", False):
                self.publish_map(key)

    def publish_map(self, key):
        # freshness gate: 최근 융합이 없으면 발행하지 않는다.
        if time.monotonic() - self.last_accepted > 1.0:
            return
        if self._map_q is None:
            return
        publisher = self._publishers_dict[key]
        if publisher.get_subscription_count() == 0:
            return
        center = self._get_map_center()
        gm = GridMap()
        gm.header.frame_id = self.map_frame
        gm.header.stamp = self._last_t if self._last_t is not None else self.get_clock().now().to_msg()
        gm.info.resolution = self._map.resolution
        actual_map_length = (self._map.cell_n - 2) * self._map.resolution
        gm.info.length_x = actual_map_length
        gm.info.length_y = actual_map_length
        # move_to() shifts by whole cells. Publish that grid's center, not the
        # continuous robot TF position, so stored terrain keeps its coordinates.
        gm.info.pose.position.x = float(center[0])
        gm.info.pose.position.y = float(center[1])
        # Heights are already in map_frame; keep visualization pose neutral.
        gm.info.pose.position.z = 0.0

        gm.info.pose.orientation.x = 0.0
        gm.info.pose.orientation.y = 0.0
        gm.info.pose.orientation.z = 0.0
        gm.info.pose.orientation.w = 1.0
        gm.layers = []
        gm.basic_layers = self.my_publishers[key]["basic_layers"]

        layers = tuple(self.my_publishers[key].get("layers", []))
        shape = (len(layers), self._map.cell_n - 2, self._map.cell_n - 2)
        data = self._map_layer_buffers.get(key)
        if data is None or data.shape != shape:
            data = np.empty(shape, dtype=np.float32)
            self._map_layer_buffers[key] = data
        self._map.get_maps_with_names_ref(layers, data)
        for layer, values in zip(layers, data):
            gm.layers.append(layer)
            gm.data.append(self._numpy_to_multiarray(values, layout="gridmap_column"))

        gm.outer_start_index = 0
        gm.inner_start_index = 0
        publisher.publish(gm)

    def _pack_2d_f32(self, arr):
        """(nx, ny) float32 배열을 라벨 붙은 Float32MultiArray(row-major)로 포장."""
        nx, ny = arr.shape
        msg = Float32MultiArray()
        msg.layout = MAL(dim=[
            MAD(label="x", size=int(nx), stride=int(nx * ny)),
            MAD(label="y", size=int(ny), stride=int(ny)),
        ], data_offset=0)
        msg.data = np.ascontiguousarray(arr, dtype=np.float32).reshape(-1).tolist()
        return msg

    def publish_policy_crop(self):
        """수정3: base 위치+yaw 기준 로컬 크롭 H_filled / M 발행 (정책 입력용)."""
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
