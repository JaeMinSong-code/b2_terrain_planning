#!/usr/bin/env python3
"""실제 GPU/ROS 메시지로 즉시 발행·TF 대기 최신 교체·stamp 보존을 검증한다."""
from pathlib import Path
import sys
import tempfile
import time
import numpy as np
import rclpy
from rclpy.serialization import serialize_message, deserialize_message
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy

package = Path(sys.argv[1]).resolve()
sys.path.insert(0, str(package / 'scripts'))
from elevation_mapping_runner import StrictElevationMappingNode
from elevation_input_adapter import ElevationInputAdapter
from generate_config import generate_configs

class Capture:
    def __init__(self):
        self.messages = []
    def get_subscription_count(self):
        return 1
    def publish(self, msg):
        self.messages.append(deserialize_message(serialize_message(msg), GridMap))


def main():
    core = Path(get_package_share_directory('elevation_mapping_cupy')) / 'config/core/core_param.yaml'
    with tempfile.TemporaryDirectory() as d:
        generate_configs(package / 'config/b2_terrain.yaml', d)
        rclpy.init(args=['--ros-args', '--params-file', str(core), '--params-file',
                        str(Path(d) / 'elevation_map_sim.yaml'),
                        '-p', 'map_frame:=elevation_odom', '-p', 'base_frame:=elevation_base'])
    mapper = StrictElevationMappingNode()
    adapter = ElevationInputAdapter()
    try:
        assert len(mapper._publishers_timers) == 0
        pub = mapper._publishers_dict['elevation_map_raw']
        assert pub.qos_profile.depth == 1
        for sub in mapper._pointcloud_subs.values():
            assert sub.qos_profile.depth == 1
            assert sub.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT
        sink = Capture()
        mapper._publishers_dict['elevation_map_raw'] = sink
        gx, gy = np.meshgrid(np.arange(-1., 1.01, .05), np.arange(-1., 1.01, .05))
        points = np.column_stack((gx.ravel(), gy.ravel(), np.full(gx.size, -.8)))
        def make_cloud(ns):
            stamp = rclpy.time.Time(nanoseconds=ns).to_msg()
            return create_cloud_xyz32(Header(stamp=stamp, frame_id='test_lidar'), points)
        def add_tf(stamp):
            for frame, height in [('test_lidar', .8), ('elevation_base', .55)]:
                tf = TransformStamped()
                tf.header.stamp = stamp
                tf.header.frame_id = 'elevation_odom'
                tf.child_frame_id = frame
                tf.transform.translation.z = height
                tf.transform.rotation.w = 1.
                mapper._tf_buffer.set_transform(tf, 'delivery_test')
        now = mapper.get_clock().now().nanoseconds
        first = make_cloud(now)
        add_tf(first.header.stamp)
        begin = time.perf_counter()
        mapper.pointcloud_callback(first, 'lidar')
        print('cold first fusion + immediate publish ms', (time.perf_counter()-begin)*1000)
        assert len(sink.messages) == 1, 'first fusion must publish before callback returns'
        assert sink.messages[-1].header.stamp == first.header.stamp
        second = make_cloud(now + 100000000)
        third = make_cloud(now + 200000000)
        mapper.pointcloud_callback(second, 'lidar')
        mapper.pointcloud_callback(third, 'lidar')
        assert len(mapper.pending_clouds) == 1
        assert mapper.pending_clouds[0][1].header.stamp == third.header.stamp
        assert len(sink.messages) == 1, 'missing TF must preserve the existing wait behavior'
        add_tf(third.header.stamp)
        begin = time.perf_counter()
        mapper.flush_inputs()
        print('warm fusion + immediate publish ms', (time.perf_counter()-begin)*1000)
        assert len(sink.messages) == 2
        assert sink.messages[-1].header.stamp == third.header.stamp
        assert sink.messages[-1].layers == ['elevation', 'is_valid', 'variance']
        mapper.flush_inputs()
        assert len(sink.messages) == 2, 'no input must not duplicate the map'
        calls = []
        adapter.process = lambda: calls.append(True)
        adapter.on_cloud(first)
        adapter.on_cloud(second)
        assert len(calls) == 2 and len(adapter.clouds) == 1
        odom = Odometry()
        odom.header.stamp = third.header.stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.orientation.w = 1.
        adapter.on_odom(odom)
        assert len(calls) == 3, 'odom arrival must retry the pending cloud immediately'
        # 실제 DDS 점군 구독 → GPU 융합 → 지도 구독 전달도 확인한다.
        mapper._publishers_dict['elevation_map_raw'] = pub
        peer = rclpy.create_node('map_delivery_probe')
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(mapper)
        executor.add_node(peer)
        received = []
        sent = {}
        cloud_subscription = next(iter(mapper._pointcloud_subs.values()))
        from sensor_msgs.msg import PointCloud2
        cloud_pub = peer.create_publisher(
            PointCloud2, cloud_subscription.topic_name, cloud_subscription.qos_profile)
        def on_map(message):
            ns = message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec
            if ns in sent:
                received.append((ns, (time.perf_counter() - sent[ns]) * 1000))
        peer.create_subscription(GridMap, pub.topic_name, on_map, 1)
        try:
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.01)
                if cloud_pub.get_subscription_count() and pub.get_subscription_count():
                    break
            assert cloud_pub.get_subscription_count() and pub.get_subscription_count()
            for index in range(20):
                ns = mapper.get_clock().now().nanoseconds
                cloud = make_cloud(ns)
                add_tf(cloud.header.stamp)
                sent[ns] = time.perf_counter()
                cloud_pub.publish(cloud)
                deadline = time.monotonic() + 1
                while len(received) <= index and time.monotonic() < deadline:
                    executor.spin_once(timeout_sec=.005)
                assert len(received) == index + 1, 'each input must deliver a map without a publish timer'
                deadline = time.monotonic() + .02
                while time.monotonic() < deadline:
                    executor.spin_once(timeout_sec=.002)
            assert len({ns for ns, _ in received}) == 20
            print('DDS cloud publish to map receive median/p95/max ms',
                  np.percentile([ms for _, ms in received], [50, 95, 100]))
        finally:
            executor.shutdown()
            peer.destroy_node()
        print('GPU/ROS immediate publication, latest-only TF wait, stamps, adapter retry and DDS passed')
    finally:
        adapter.destroy_node()
        mapper.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
