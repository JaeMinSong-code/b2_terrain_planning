#!/usr/bin/env python3
"""입력 GridMap의 구조를 보존하고 높이·분산·유효성만 평탄화한다.

최근 센서 관측의 평면 높이를 따라 odometry 드리프트를 반영한다.
발 링크 원점과 센서 지면의 고정 오프셋만 최초 안정 접촉으로 맞춘다.
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
from geometry_msgs.msg import PointStamped


class PlaneHeightTracker:
    """초기 링크 높이 보정은 유지하고 매 지도에서 관측 높이 변화만 반영한다."""
    def __init__(self):
        self.offset = None

    def update(self, observed_height, initial_support_height):
        if not math.isfinite(observed_height):
            raise ValueError('non-finite observed plane height')
        if self.offset is None:
            if initial_support_height is None or not math.isfinite(initial_support_height):
                raise ValueError('waiting for stable initial support height')
            self.offset = initial_support_height - observed_height
        return observed_height + self.offset


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
        self.support_height = None
        self.plane = PlaneHeightTracker()
        self.state = None
        self.last_stamp = None
        self.last_map_stamp = None
        self.epoch = None
        self.observations = {}
        self.pending_map = None
        self.calibration = deque()
        input_topic = self.declare_parameter(
            'input_topic', '/elevation_mapping_node/elevation_map_raw').value
        output_topic = self.declare_parameter(
            'output_topic', '/terrain_planner/elevation_map_flat').value
        if input_topic == output_topic:
            raise ValueError('input and output topics must differ')
        self.publisher = self.create_publisher(
            GridMap, output_topic, 1)
        self.create_subscription(LocomotionState, '/robot/locomotion_state',
                                 self.on_state, qos_profile_sensor_data)
        self.create_subscription(GridMap, input_topic, self.on_map, qos_profile_sensor_data)
        self.create_subscription(PointStamped, '/elevation/observed_plane', self.on_plane, 10)
        self.get_logger().info('Flat map: waiting for four estimated support contacts')

    def reset_plane(self):
        self.support_height = None
        self.plane = PlaneHeightTracker()
        self.calibration.clear()
        self.last_map_stamp = None
        self.observations.clear()
        self.pending_map = None

    def on_plane(self, message):
        if not self.fresh(message.header, 'elevation_odom') or not math.isfinite(message.point.z):
            return
        stamp = rclpy.time.Time.from_msg(message.header.stamp).nanoseconds
        self.observations[stamp] = message.point.z
        self.observations = dict(sorted(self.observations.items())[-20:])
        if self.pending_map is not None:
            self.on_map(self.pending_map)

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
        if self.plane.offset is None:
            self.support_height = None
            if not all(state.contact_estimated):
                self.calibration.clear()
                return
            heights = [foot.z for foot in state.foot_position]
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
            self.support_height = mean

    def on_map(self, message):
        if self.state is None or not self.fresh(self.state.header):
            return
        if not self.fresh(message.header, 'elevation_odom', 0.25):
            return
        stamp = rclpy.time.Time.from_msg(message.header.stamp).nanoseconds
        if self.last_map_stamp is not None and stamp <= self.last_map_stamp:
            return
        self.pending_map = message
        # DDS 채널 도착 순서와 무관하게 동일 source stamp끼리만 결합한다.
        if stamp not in self.observations:
            return
        try:
            observed = self.observations[stamp]
            first = self.plane.offset is None
            height = self.plane.update(observed, self.support_height)
            result = flatten_map(message, height)
            self.publisher.publish(result)
            self.last_map_stamp = stamp
            self.pending_map = None
            if first:
                self.get_logger().info(
                    f'Flat map tracking observations: offset={self.plane.offset:.6f} m')
            self.get_logger().info(
                f'Flat plane height={height:.6f} m, observed={observed:.6f} m',
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
