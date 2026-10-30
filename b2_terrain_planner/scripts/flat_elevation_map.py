#!/usr/bin/env python3
"""입력 GridMap의 구조를 보존하고 높이·분산·유효성만 평탄화한다.

평면 높이는 확인 지지발 중심 z에서 구 반경을 뺀 지면 높이를 추적한다.
지지발은 유효 관측 나이가 0인 유일한 지면 측정이므로, odometry z 드리프트가
지도 융합 지연으로 남기는 지도-발 괴리를 매 주기 재정착으로 제거한다.
확인 지지발이 2개 미만인 구간은 직전 높이를 유지한다.
"""
import math
import copy
import signal
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from b2_terrain_planner.msg import LocomotionState
from rclpy.executors import ExternalShutdownException
from grid_map_msgs.msg import GridMap


class SupportPlaneTracker:
    """확인 지지발에서 변환한 지면 높이 중앙값으로 평면을 추적한다.

    초기 높이는 안정 기립 보정이 설정하고, 이후에는 변화율 제한으로만 이동해
    접촉 추정 오염 한 번에 지도가 점프하지 않게 한다. odometry z 드리프트
    (관측상 ~1.4mm/s)는 제한 안에서 그대로 따라간다.
    """

    def __init__(self, max_rate_mps=0.01, max_step_sec=0.1):
        self.height = None
        self.last_stamp_ns = None
        self.max_rate = max_rate_mps
        self.max_step_sec = max_step_sec

    def initialize(self, stamp_ns, height):
        if not math.isfinite(height):
            raise ValueError('non-finite initial support height')
        self.height = height
        self.last_stamp_ns = stamp_ns

    def update(self, stamp_ns, support_heights):
        """발 반경을 이미 뺀 지면 z 목록으로 높이를 갱신한다. 초기화 전이나 빈 목록은 유지."""
        if self.height is None:
            return None
        if self.last_stamp_ns is not None and stamp_ns <= self.last_stamp_ns:
            return self.height
        heights = sorted(z for z in support_heights if math.isfinite(z))
        if len(heights) < 2:
            # 지지 2개 미만(스윙 전환·접촉 깜빡임)은 직전 평면을 유지한다.
            self.last_stamp_ns = stamp_ns
            return self.height
        middle = len(heights) // 2
        if len(heights) % 2:
            median = heights[middle]
        else:
            median = 0.5 * (heights[middle - 1] + heights[middle])
        dt = self.max_step_sec
        if self.last_stamp_ns is not None:
            dt = min(dt, (stamp_ns - self.last_stamp_ns) * 1e-9)
        limit = self.max_rate * max(0.0, dt)
        self.height += min(limit, max(-limit, median - self.height))
        self.last_stamp_ns = stamp_ns
        return self.height


def flatten_map(message, height):
    """메타데이터·원형 버퍼·다른 레이어·원본 메시지를 보존한 평면 복사본."""
    if not math.isfinite(height) or 'elevation' not in message.layers:
        raise ValueError('finite height and elevation layer required')
    if len(message.layers) != len(message.data):
        raise ValueError('layer/data count mismatch')
    cells = len(message.data[message.layers.index('elevation')].data)
    if cells == 0:
        raise ValueError('empty elevation layer')
    result = copy.deepcopy(message)
    for layer, value in (('elevation', height), ('variance', 0.0), ('is_valid', 1.0)):
        if layer not in result.layers:
            continue
        data = result.data[result.layers.index(layer)]
        if len(data.data) != cells:
            raise ValueError('layer size mismatch')
        data.data = [value] * cells
    return result


class FlatElevationMap(Node):
    def __init__(self):
        super().__init__('flat_elevation_map')
        self.state = None
        self.last_stamp = None
        self.last_map_stamp = None
        self.epoch = None
        self.calibration = deque()
        input_topic = self.declare_parameter(
            'input_topic', '/elevation_mapping_node/elevation_map_raw').value
        output_topic = self.declare_parameter(
            'output_topic', '/terrain_planner/elevation_map_flat').value
        self.foot_radius = self.declare_parameter('foot_radius', 0.032).value
        rate_limit = self.declare_parameter('plane_rate_limit_mps', 0.01).value
        if input_topic == output_topic:
            raise ValueError('input and output topics must differ')
        if not math.isfinite(rate_limit) or rate_limit <= 0:
            raise ValueError('plane_rate_limit_mps must be positive and finite')
        self.plane = SupportPlaneTracker(rate_limit)
        self.publisher = self.create_publisher(
            GridMap, output_topic, 1)
        self.create_subscription(LocomotionState, '/robot/locomotion_state',
                                 self.on_state, qos_profile_sensor_data)
        self.create_subscription(GridMap, input_topic, self.on_map, qos_profile_sensor_data)
        self.get_logger().info('Flat map: waiting for four estimated support contacts')

    def reset_plane(self):
        self.plane = SupportPlaneTracker(self.plane.max_rate, self.plane.max_step_sec)
        self.calibration.clear()
        self.last_map_stamp = None

    def fresh(self, header, frame='odom', max_age=0.25):
        stamp = rclpy.time.Time.from_msg(header.stamp)
        age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
        return header.frame_id == frame and 0 <= age <= max_age

    def on_state(self, state):
        if not self.fresh(state.header):
            return
        stamp = rclpy.time.Time.from_msg(state.header.stamp).nanoseconds
        if self.last_stamp is not None:
            if stamp == self.last_stamp:
                return
            if stamp < self.last_stamp or stamp - self.last_stamp > 1_000_000_000:
                self.reset_plane()
        if state.gait_clock_valid:
            if self.epoch is not None and self.epoch != state.gait_epoch_ns:
                self.reset_plane()
            self.epoch = state.gait_epoch_ns
        self.last_stamp = stamp
        self.state = state
        if self.plane.height is None:
            self.calibrate(state, stamp)
            return
        supports = [foot.z - self.foot_radius for foot, confirmed, estimated
                    in zip(state.foot_position, state.contact_confirmed,
                           state.contact_estimated)
                    if confirmed and estimated]
        self.plane.update(stamp, supports)

    def calibrate(self, state, stamp):
        """기립 중 네 발이 같은 높이로 안정될 때 초기 평면을 설정한다."""
        if not all(state.contact_estimated):
            self.calibration.clear()
            return
        heights = [foot.z - self.foot_radius for foot in state.foot_position]
        if not all(math.isfinite(value) for value in heights):
            return
        if max(heights) - min(heights) > 0.01:
            self.calibration.clear()
            return
        mean = sum(heights) / 4
        self.calibration.append((stamp, mean))
        while self.calibration and stamp - self.calibration[0][0] > 700_000_000:
            self.calibration.popleft()
        if stamp - self.calibration[0][0] < 500_000_000:
            return
        levels = [entry[1] for entry in self.calibration]
        if max(levels) - min(levels) > 0.002:
            return
        self.plane.initialize(stamp, mean)
        self.get_logger().info(
            f'Flat ground plane anchored below foot centers: height={mean:.6f} m')

    def on_map(self, message):
        if self.state is None or not self.fresh(self.state.header):
            return
        if not self.fresh(message.header, 'elevation_odom', 0.25):
            return
        stamp = rclpy.time.Time.from_msg(message.header.stamp).nanoseconds
        if self.last_map_stamp is not None and stamp <= self.last_map_stamp:
            return
        if self.plane.height is None:
            return
        try:
            result = flatten_map(message, self.plane.height)
            self.publisher.publish(result)
            self.last_map_stamp = stamp
            self.get_logger().info(
                f'Flat plane height={self.plane.height:.6f} m (ground datum, foot radius removed)',
                throttle_duration_sec=2.0)
        except ValueError as error:
            self.get_logger().warning(str(error), throttle_duration_sec=2.0)


def main():
    rclpy.init()
    node = FlatElevationMap()
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
