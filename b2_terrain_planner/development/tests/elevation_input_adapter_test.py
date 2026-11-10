#!/usr/bin/env python3
"""원격 수신 시각 지원과 최신 점군 즉시 처리의 병합 계약을 검증한다."""
from collections import deque
from pathlib import Path
from types import SimpleNamespace
import sys
import time
import unittest
from unittest.mock import Mock, patch

import numpy as np
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2
from tf2_ros import TransformException

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'scripts'))
import elevation_input_adapter as adapter


class ElevationInputAdapterTest(unittest.TestCase):
    def fixture(self, receive=True, stamp=231):
        cloud = PointCloud2()
        cloud.header.stamp = Time(sec=stamp)
        cloud.header.frame_id = 'lidar'
        clock = Mock()
        clock.now.return_value.nanoseconds = 100000000000
        clock.now.return_value.to_msg.return_value = Time(sec=100)
        node = SimpleNamespace(
            cfg={'use_receive_time': receive, 'max_age': 1.0, 'queue_wait': .35,
                 'max_odom_gap': .3, 'body_frame': 'base_link',
                 'body_frame_alias': 'robot_center', 'extrinsic_parent': '',
                 'extrinsic_child': '', 'extrinsic_child_from_cloud': False},
            reset_pending=False, last_good=time.monotonic(),
            clouds=deque([(time.monotonic(), Time(sec=100), cloud)], maxlen=1),
            poses=deque([
                adapter.Pose(stamp - .1, 'odom', 'base_link', np.zeros(3), np.array([0., 0., 0., 1.])),
                adapter.Pose(stamp + .1, 'odom', 'base_link', np.array([2., 0., 0.]), np.array([0., 0., 0., 1.]))]),
            get_clock=Mock(return_value=clock), status=Mock(),
            points_pub=Mock(), tf_pub=Mock(), tf_buffer=Mock(),
            lookup=Mock(return_value=(np.zeros(3), np.array([0., 0., 0., 1.]))))
        return node, cloud

    def process(self, node):
        with patch.object(adapter, 'decode_xyz', return_value=np.array([[1., 2., 3.]])):
            adapter.ElevationInputAdapter.process(node)

    def test_receive_time_preserves_source_alignment(self):
        node, cloud = self.fixture()
        self.process(node)
        output = node.points_pub.publish.call_args.args[0]
        transforms = node.tf_pub.publish.call_args.args[0].transforms
        self.assertEqual(cloud.header.stamp.sec, 231)
        self.assertEqual(output.header.stamp.sec, 100)
        self.assertTrue(all(t.header.stamp == output.header.stamp for t in transforms))
        self.assertAlmostEqual(transforms[0].transform.translation.x, 1.0)
        self.assertTrue(all(call.args[2] == cloud.header.stamp for call in node.lookup.call_args_list))
        self.assertEqual(len(node.clouds), 0)

    def test_original_time_mode_keeps_timestamp(self):
        node, cloud = self.fixture(receive=False, stamp=100)
        self.process(node)
        self.assertEqual(node.points_pub.publish.call_args.args[0].header.stamp, cloud.header.stamp)

    def test_original_time_mode_rejects_future_input(self):
        node, _ = self.fixture(receive=False)
        self.process(node)
        node.points_pub.publish.assert_not_called()
        self.assertEqual(len(node.clouds), 0)

    def test_receive_age_expires(self):
        node, cloud = self.fixture()
        node.clouds[0] = (time.monotonic() - 2, Time(sec=100), cloud)
        self.process(node)
        node.points_pub.publish.assert_not_called()
        self.assertEqual(len(node.clouds), 0)

    def test_missing_odometry_waits(self):
        node, _ = self.fixture()
        node.poses.clear()
        self.process(node)
        node.points_pub.publish.assert_not_called()
        self.assertEqual(len(node.clouds), 1)

    def test_latest_cloud_is_processed_immediately(self):
        node, cloud = self.fixture()
        node.clouds.clear()
        node.cloud_frame = None
        node.last_cloud_stamp = None
        node.process = Mock()
        adapter.ElevationInputAdapter.on_cloud(node, cloud)
        second = PointCloud2()
        second.header.frame_id = 'lidar'
        second.header.stamp = Time(sec=232)
        adapter.ElevationInputAdapter.on_cloud(node, second)
        self.assertEqual(node.process.call_count, 2)
        self.assertEqual(len(node.clouds), 1)
        self.assertIs(node.clouds[0][2], second)
        self.assertEqual(node.clouds[0][1].sec, 100)

    def test_body_alias_prefers_real_tf_and_never_aliases_sensor(self):
        node, _ = self.fixture()
        transform = TransformStamped()
        transform.transform.translation.x = .4
        transform.transform.rotation.w = 1.
        node.tf_buffer.lookup_transform.return_value = transform
        lookup = adapter.ElevationInputAdapter.lookup
        position, _ = lookup(node, 'robot_center', 'base_link', Time(sec=231))
        self.assertAlmostEqual(position[0], .4)
        node.tf_buffer.lookup_transform.side_effect = TransformException('missing')
        position, attitude = lookup(node, 'robot_center', 'base_link', Time(sec=231))
        np.testing.assert_array_equal(position, np.zeros(3))
        np.testing.assert_array_equal(attitude, [0., 0., 0., 1.])
        with self.assertRaises(TransformException):
            lookup(node, 'robot_center', 'base_link', Time(sec=231), sensor=True)


if __name__ == '__main__':
    unittest.main()
