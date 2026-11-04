#!/usr/bin/env python3
"""실제 CuPy 초기화기의 높이·범위·관측 보존·일회성·후속 융합을 검증한다."""
import math
from pathlib import Path
import sys
import tempfile

import cupy as cp
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header
from std_srvs.srv import Trigger

package = Path(sys.argv[1]).resolve()
sys.path.insert(0, str(package / 'scripts'))
from elevation_mapping_runner import StrictElevationMappingNode
from generate_config import generate_configs
from elevation_mapping_cupy.elevation_mapping_node import ElevationMappingNode


def main():
    core = Path(get_package_share_directory('elevation_mapping_cupy')) / 'config/core/core_param.yaml'
    with tempfile.TemporaryDirectory(prefix='b2-initializer-config-') as directory:
        generate_configs(package / 'config/b2_terrain.yaml', directory)
        rclpy.init(args=['--ros-args', '--params-file', str(core), '--params-file',
                        str(Path(directory) / 'elevation_map_sim.yaml'),
                        '-p', 'map_frame:=elevation_odom', '-p', 'base_frame:=elevation_base'])
    mapper = StrictElevationMappingNode()
    try:
        assert mapper.use_initializer_at_start and mapper.initializer_pending
        stamp = mapper.get_clock().now().to_msg()
        header = Header(stamp=stamp, frame_id='test_lidar')
        gx, gy = np.meshgrid(np.arange(-1.0, 1.01, 0.05), np.arange(-1.0, 1.01, 0.05))
        ground_z = -0.031
        sensor_z = 0.8
        points = np.column_stack((gx.ravel(), gy.ravel(),
                                  np.full(gx.size, ground_z - sensor_z)))
        cloud = create_cloud_xyz32(header, points)
        mapper.try_initialize_from_ground(cloud)
        assert mapper.initializer_pending, 'missing TF must defer only initialization'
        assert int(cp.count_nonzero(mapper._map.elevation_map[2]).item()) == 0
        yaw = 0.4
        for frame, xyz, angle in [('test_lidar', [0.0, 0.0, sensor_z], 0.0),
                                  ('elevation_base', [0.1, -0.1, 0.55], yaw)]:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = 'elevation_odom'
            tf.child_frame_id = frame
            tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = xyz
            tf.transform.rotation.z = math.sin(angle / 2)
            tf.transform.rotation.w = math.cos(angle / 2)
            mapper._tf_buffer.set_transform(tf, 'initializer_test')
        mapper._last_t = stamp
        mapper.pose_update()
        m = mapper._map
        center = cp.asnumpy(m.center)
        # CuPy 배열은 행=Y, 열=X이므로 월드 좌표와 축 순서가 다르다.
        col, row = ((np.array([0.0, 0.0]) - center[:2]) / m.resolution + m.cell_n / 2).astype(int)
        m.elevation_map[0, row, col] = 0.42
        m.elevation_map[1, row, col] = 0.004
        m.elevation_map[2, row, col] = 1.0
        before = m.elevation_map.copy()
        mapper.last_initializer_attempt = -math.inf
        mapper.try_initialize_from_ground(create_cloud_xyz32(header, []))
        assert mapper.initializer_pending, 'empty cloud must not create a zero-height floor'
        assert bool(cp.array_equal(before, m.elevation_map).item())
        mapper.last_initializer_attempt = -math.inf
        mapper.try_initialize_from_ground(cloud)
        assert not mapper.initializer_pending
        valid = m.elevation_map[2] > 0.5
        seeded = valid & (before[2] <= 0.5)
        count = int(cp.count_nonzero(seeded).item())
        expected = mapper.initialize_patch_length * mapper.initialize_patch_width / m.resolution**2
        assert 0.9 * expected < count < 1.1 * expected, count
        assert bool(cp.array_equal(before[:3, row, col],
                                   m.elevation_map[:3, row, col]).item()), 'observed obstacle changed'
        assert float(cp.max(cp.abs(m.elevation_map[0, seeded] + center[2] - ground_z)).item()) < 1e-5
        rows, cols = np.nonzero(cp.asnumpy(seeded))
        xy = (np.column_stack((cols, rows)) - m.cell_n / 2) * m.resolution + center[:2]
        c, s = math.cos(yaw), math.sin(yaw)
        local = (xy - [0.1, -0.1]) @ np.array([[c, -s], [s, c]])
        assert np.max(np.abs(local[:, 0])) <= mapper.initialize_patch_length / 2 + 1e-6
        assert np.max(np.abs(local[:, 1])) <= mapper.initialize_patch_width / 2 + 1e-6
        # 비정방형 패치가 회전된 몸통의 앞뒤·좌우 범위를 모두 채워야 한다.
        for axis, size in enumerate((mapper.initialize_patch_length, mapper.initialize_patch_width)):
            assert np.min(local[:, axis]) < -size / 2 + 2 * m.resolution
            assert np.max(local[:, axis]) > size / 2 - 2 * m.resolution
        saved = m.elevation_map.copy()
        mapper.last_initializer_attempt = -math.inf
        mapper.try_initialize_from_ground(cloud)
        assert bool(cp.array_equal(saved, m.elevation_map).item()), 'initializer repeated'
        # 일반 센서 경로로 초기화 셀도 새 관측을 받는지 확인한다.
        points[:, 2] += 0.01
        update = create_cloud_xyz32(header, points)
        for _ in range(3):
            ElevationMappingNode.pointcloud_callback(mapper, update, 'lidar')
        fused = seeded & (m.elevation_map[2] > 0.5) & (m.elevation_map[1] < saved[1])
        assert int(cp.count_nonzero(fused).item()) > 20, 'seeded cells did not accept LiDAR updates'
        assert float(cp.max(cp.abs(m.elevation_map[0, fused] - saved[0, fused])).item()) > 0.001
        response = mapper.handle_clear_map(Trigger.Request(), Trigger.Response())
        assert response.success and mapper.initializer_pending
        assert int(cp.count_nonzero(m.elevation_map[2]).item()) == 0
        mapper.use_initializer_at_start = False
        response = mapper.handle_clear_map(Trigger.Request(), Trigger.Response())
        assert response.success and not mapper.initializer_pending
        print(f'PASS GPU ground initializer: {count} cells, observed floor z, yaw/size bounds, '
              'obstacle preserved, one-shot, deferred TF/cloud, LiDAR updates, clear/disable')
    finally:
        mapper.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
