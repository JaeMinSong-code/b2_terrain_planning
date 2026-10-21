#!/usr/bin/env python3
"""평탄화가 원본 지도 메타데이터·원형 버퍼·추가 레이어를 보존하는지 검사한다."""
import copy
import math
from pathlib import Path
import sys
import unittest

from grid_map_msgs.msg import GridMap
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from flat_elevation_map import flatten_map, PlaneHeightTracker, FlatElevationMap
from elevation_common import flat_ground_height
import numpy as np
from types import SimpleNamespace
from unittest.mock import Mock
from geometry_msgs.msg import PointStamped


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

    def test_current_cloud_tracks_drift_without_accumulated_map_or_foot_feedback(self):
        ground = np.column_stack([np.linspace(-1, 1, 40), np.zeros(40), np.full(40, -.52)])
        tracker = PlaneHeightTracker()
        first = tracker.update(flat_ground_height(ground, [0, 0, 0], 2), -.485)
        drifted = ground + [0, 0, .008]
        updated = tracker.update(flat_ground_height(drifted, [0, 0, .008], 2), -.40)
        self.assertAlmostEqual(updated-first, .008, places=6)
        # 원본 누적 지도가 과거 높이에 있어도 출력 전체는 새 관측 평면을 따른다.
        source = self.source()
        source.data[1].data = [-.52] * 12
        result = flatten_map(source, updated)
        self.assertTrue(all(abs(z-updated) < 1e-6 for z in result.data[1].data))
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
        node = SimpleNamespace(state=SimpleNamespace(header=None),
                               support_height=-.485, plane=PlaneHeightTracker(),
                               last_map_stamp=None, pending_map=None, observations={},
                               publisher=Mock(), fresh=lambda *args: True,
                               get_logger=lambda: Mock())
        node.on_map = lambda msg: FlatElevationMap.on_map(node, msg)
        return node

    def test_map_waits_for_matching_cloud_stamp_and_does_not_republish(self):
        node = self.harness()
        message = self.source()
        node.on_map(message)
        node.publisher.publish.assert_not_called()
        plane = PointStamped()
        plane.header = copy.deepcopy(message.header)
        plane.header.stamp.nanosec += 1
        plane.point.z = -.52
        FlatElevationMap.on_plane(node, plane)
        node.publisher.publish.assert_not_called()
        plane.header = copy.deepcopy(message.header)
        FlatElevationMap.on_plane(node, plane)
        node.publisher.publish.assert_called_once()
        self.assertAlmostEqual(node.publisher.publish.call_args.args[0].data[1].data[0], -.485)
        node.on_map(message)
        node.publisher.publish.assert_called_once()

    def test_stale_plane_and_session_reset_do_not_reuse_old_offset(self):
        node = self.harness()
        node.fresh = lambda *args: False
        plane = PointStamped()
        plane.point.z = -.52
        FlatElevationMap.on_plane(node, plane)
        self.assertEqual(node.observations, {})
        node.plane.update(-.52, -.485)
        node.calibration = []
        FlatElevationMap.reset_plane(node)
        self.assertIsNone(node.plane.offset)
        self.assertIsNone(node.support_height)


if __name__ == '__main__':
    unittest.main()
