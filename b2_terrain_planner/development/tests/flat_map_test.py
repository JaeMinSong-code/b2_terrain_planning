#!/usr/bin/env python3
"""평탄화가 원본 지도 메타데이터·원형 버퍼·추가 레이어를 보존하는지 검사한다."""
import copy
import math
from pathlib import Path
import sys
import unittest

from grid_map_msgs.msg import GridMap
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'scripts'))
from flat_elevation_map import flatten_map, SupportPlaneTracker, FlatElevationMap
from elevation_common import flat_ground_height
import numpy as np
from types import SimpleNamespace
from unittest.mock import Mock


class FlatMapTest(unittest.TestCase):
    def source(self):
        message = GridMap()
        message.header.frame_id = 'elevation_odom'
        message.header.stamp.sec = 123
        message.header.stamp.nanosec = 456
        message.info.resolution = 0.1
        message.info.length_x = 0.3
        message.info.length_y = 0.4
        message.info.pose.position.x = 1.23
        message.info.pose.position.y = -4.56
        message.info.pose.orientation.w = 1.0
        message.outer_start_index = 2
        message.inner_start_index = 1
        message.layers = ['variance', 'elevation', 'is_valid', 'color']
        message.basic_layers = ['elevation']
        for values in ([0.5]*12, [float('nan')]*12, [0.0]*12, list(range(12))):
            data = Float32MultiArray(data=[float(value) for value in values])
            data.layout.dim = [MultiArrayDimension(label='column_index', size=4, stride=12),
                               MultiArrayDimension(label='row_index', size=3, stride=3)]
            message.data.append(data)
        return message

    def test_geometry_buffer_timestamp_and_original_are_preserved(self):
        source = self.source()
        result = flatten_map(source, -0.48)
        self.assertEqual(result.header, source.header)
        self.assertEqual(result.info, source.info)
        self.assertEqual(result.layers, source.layers)
        self.assertEqual(result.basic_layers, source.basic_layers)
        self.assertEqual(result.outer_start_index, source.outer_start_index)
        self.assertEqual(result.inner_start_index, source.inner_start_index)
        for index in range(4):
            self.assertEqual(result.data[index].layout, source.data[index].layout)
        for name, value in [('elevation', -0.48), ('variance', 0), ('is_valid', 1)]:
            for cell in result.data[result.layers.index(name)].data:
                self.assertAlmostEqual(cell, value, places=6)
        self.assertEqual(result.data[3], source.data[3])
        self.assertTrue(all(math.isnan(cell) for cell in source.data[1].data))
        self.assertEqual(list(source.data[2].data), [0]*12)
        # 영역 이동도 입력 값 그대로 전달한다.
        shifted = copy.deepcopy(source)
        shifted.info.pose.position.x += 0.3
        shifted.outer_start_index = 0
        output = flatten_map(shifted, -0.48)
        self.assertEqual(output.info, shifted.info)
        self.assertEqual(output.outer_start_index, 0)

    def test_missing_or_invalid_layers_are_rejected(self):
        source = self.source()
        with self.assertRaises(ValueError):
            flatten_map(source, float('nan'))
        source.data[0].data = [0.0]
        with self.assertRaises(ValueError):
            flatten_map(source, 0.0)
        source.layers[1] = 'unexpected'
        with self.assertRaises(ValueError):
            flatten_map(source, 0.0)

    def test_support_feet_datum_tracks_drift_with_rate_limit(self):
        tracker = SupportPlaneTracker(max_rate_mps=.01)
        self.assertIsNone(tracker.update(0, [-.485, -.485]))  # 초기화 전에는 유지
        tracker.initialize(0, -.485)
        # 드리프트 속도(~1.4mm/s)는 제한 안에서 그대로 따라간다.
        height = tracker.update(100_000_000, [-.4852, -.4851, -.4851])
        self.assertAlmostEqual(height, -.4851, places=6)
        # 접촉 오염으로 한 발이 튀어도 중앙값과 변화율 제한이 점프를 막는다.
        height = tracker.update(200_000_000, [-.4851, -.4851, -.30, -.4850])
        self.assertAlmostEqual(height, -.48505, places=6)
        jumped = tracker.update(300_000_000, [-.30, -.31])
        self.assertAlmostEqual(jumped, -.48505 + .001, places=6)
        # 지지 2개 미만이나 시각 역행은 직전 높이를 유지한다.
        self.assertAlmostEqual(tracker.update(400_000_000, [-.40]), jumped, places=9)
        self.assertAlmostEqual(tracker.update(300_000_000, [-.40, -.40]), jumped, places=9)
        # 상태 공백 후에는 dt 상한으로 한 번에 크게 움직이지 않는다.
        held = tracker.update(5_000_000_000, [-.52, -.52])
        self.assertAlmostEqual(held, jumped - .001, places=6)
        # 원본 누적 지도가 과거 높이에 있어도 출력 전체는 지지발 기준을 따른다.
        source = self.source()
        source.data[1].data = [-.52] * 12
        result = flatten_map(source, held)
        self.assertTrue(all(abs(z-held) < 1e-6 for z in result.data[1].data))
        self.assertEqual(result.info, source.info)

    def test_cloud_plane_rejects_missing_ground_and_ignores_body_outliers(self):
        ground = np.column_stack([np.linspace(-1, 1, 40), np.zeros(40), np.full(40, -.52)])
        points = np.vstack([ground, [[0, 0, 0], [0, 0, 1], [10, 0, -1],
                                    [float('nan'), 0, -.52], [0, 0, -1]]])
        self.assertAlmostEqual(flat_ground_height(points, [0, 0, 0], 2), -.52)
        with self.assertRaises(ValueError):
            flat_ground_height(ground[:5], [0, 0, 0], 2)
        with self.assertRaises(ValueError):
            flat_ground_height(ground+10, [0, 0, 0], 2)

    def harness(self):
        from collections import deque
        plane = SupportPlaneTracker()
        plane.initialize(0, -.485)
        node = SimpleNamespace(state=SimpleNamespace(header=None), plane=plane,
                               last_map_stamp=None, calibration=deque(),
                               publisher=Mock(), fresh=lambda *args: True,
                               get_logger=lambda: Mock())
        node.on_map = lambda msg: FlatElevationMap.on_map(node, msg)
        return node

    def test_map_publishes_support_datum_and_does_not_republish(self):
        node = self.harness()
        message = self.source()
        node.on_map(message)
        node.publisher.publish.assert_called_once()
        self.assertAlmostEqual(node.publisher.publish.call_args.args[0].data[1].data[0], -.485)
        node.on_map(message)  # 같은 지도 stamp는 재발행하지 않는다.
        node.publisher.publish.assert_called_once()
        newer = copy.deepcopy(message)
        newer.header.stamp.nanosec += 1
        node.plane = SupportPlaneTracker()  # 초기화 전에는 발행하지 않는다.
        node.on_map(newer)
        node.publisher.publish.assert_called_once()

    def test_foot_centers_are_converted_once_during_calibration_and_tracking(self):
        from b2_terrain_planner.msg import LocomotionState
        for radius in (0.032, 0.05):
            node = self.harness()
            node.foot_radius = radius
            node.plane = SupportPlaneTracker()
            node.last_stamp = None
            node.epoch = None
            node.calibrate = lambda state, stamp: FlatElevationMap.calibrate(node, state, stamp)
            state = LocomotionState()
            state.contact_estimated = [True] * 4
            state.contact_confirmed = [True] * 4
            for foot in state.foot_position:
                foot.z = 0.2 + radius
            for tick in range(7):
                state.header.stamp.nanosec = tick * 100_000_000
                FlatElevationMap.on_state(node, state)
            self.assertAlmostEqual(node.plane.height, 0.2)
            state.header.stamp.nanosec = 700_000_000
            for foot in state.foot_position:
                foot.z += 0.0005
            FlatElevationMap.on_state(node, state)
            self.assertAlmostEqual(node.plane.height, 0.2005)
            node.on_map(self.source())
            cells = node.publisher.publish.call_args.args[0].data[1].data
            self.assertTrue(all(abs(z - 0.2005) < 1e-6 for z in cells))

    def test_session_reset_clears_support_datum(self):
        node = self.harness()
        node.last_map_stamp = 7
        FlatElevationMap.reset_plane(node)
        self.assertIsNone(node.plane.height)
        self.assertIsNone(node.last_map_stamp)
        message = self.source()
        node.on_map(message)
        node.publisher.publish.assert_not_called()


if __name__ == '__main__':
    unittest.main()
