#!/usr/bin/env python3
"""격리 ROS domain/GPU에서 초기화 TF, 원본 보존, 1회 실행과 clear를 검증한다.

사용: elevation_env.sh 로드 후 python initializer_gpu_test.py <b2_terrain_planner 경로>
"""
import math
from pathlib import Path
import sys
import time

import cupy as cp
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from b2_terrain_planner.msg import LocomotionState
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

package = Path(sys.argv[1]).resolve()
sys.path.insert(0, str(package / 'scripts'))
from elevation_input_adapter import ElevationInputAdapter
from elevation_mapping_runner import StrictElevationMappingNode


def main():
    core = Path(get_package_share_directory('elevation_mapping_cupy')) / 'config/core/core_param.yaml'
    rclpy.init(args=['--ros-args', '--params-file', str(core), '--params-file',
                    str(package / 'config/elevation_map_sim.yaml'),
                    '-p', 'map_frame:=elevation_odom', '-p', 'base_frame:=elevation_base'])
    mapper = StrictElevationMappingNode()
    adapter = ElevationInputAdapter()
    observer = rclpy.create_node('initializer_tf_check')
    messages = []
    subscription = observer.create_subscription(TFMessage, '/elevation/tf', messages.append, 10)
    try:
        assert mapper.use_initializer_at_start and mapper.initializer_pending
        assert mapper.initialize_method == 'linear'
        assert list(mapper.initialize_frame_id) == ['FL_foot', 'FR_foot', 'RL_foot', 'RR_foot']
        assert list(mapper.initialize_tf_offset) == [0.0] * 4
        assert mapper.param.dilation_size_initialize == 2

        # TF 미준비는 초기화만 대기하며 지도 내용을 변경하지 않는다.
        mapper.try_initialize_from_feet(mapper.get_clock().now().to_msg())
        assert mapper.initializer_pending
        assert int(cp.count_nonzero(mapper._map.elevation_map[2]).item()) == 0

        state = LocomotionState()
        state.header.frame_id = 'odom'
        feet = [(0.32, 0.19), (0.32, -0.19), (-0.34, 0.19), (-0.34, -0.19)]
        for foot, (x, y) in zip(state.foot_position, feet):
            foot.x, foot.y, foot.z = x, y, 0.03
        odom = Odometry()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.orientation.w = 1.0
        deadline = time.monotonic() + 3.0
        while not messages and time.monotonic() < deadline:
            stamp = adapter.get_clock().now().to_msg()
            odom.header.stamp = stamp
            state.header.stamp = stamp
            adapter.on_odom(odom)
            adapter.on_state(state)
            rclpy.spin_once(observer, timeout_sec=0.05)
        assert messages, 'no isolated foot TF published'
        frames = messages[-1].transforms
        assert [tf.child_frame_id for tf in frames] == list(mapper.initialize_frame_id)
        for tf, foot in zip(frames, state.foot_position):
            assert tf.header.frame_id == 'elevation_odom'
            assert all(getattr(tf.transform.translation, axis) == getattr(foot, axis)
                       for axis in ('x', 'y', 'z'))
            mapper._tf_buffer.set_transform(tf, 'initializer_test')
        stamp = frames[0].header.stamp
        mapper._map.move_to(np.array([0.1, -0.1, 0.55]), np.eye(3))
        m = mapper._map
        center = cp.asnumpy(m.center)
        # 초기화 범위 안의 기존 장애물 높이도 지우면 안 된다.
        idx = ((np.array([0.0, 0.0]) - center[:2]) / m.resolution + m.cell_n / 2).astype(int)
        m.elevation_map[0, idx[0], idx[1]] = 0.42
        m.elevation_map[1, idx[0], idx[1]] = 0.004
        m.elevation_map[2, idx[0], idx[1]] = 1.0
        mapper.last_initializer_attempt = -math.inf
        mapper.try_initialize_from_feet(stamp)
        assert not mapper.initializer_pending
        valid = m.elevation_map[2] > 0.5
        count = int(cp.count_nonzero(valid).item())
        assert 10 < count < m.cell_n * m.cell_n // 2, count
        assert abs(float(m.elevation_map[0, idx[0], idx[1]].item()) - 0.42) < 1e-6
        assert abs(float(m.elevation_map[1, idx[0], idx[1]].item()) - 0.004) < 1e-6
        seeded = valid.copy()
        seeded[idx[0], idx[1]] = False
        assert float(cp.max(cp.abs(m.elevation_map[0, seeded] + center[2] - 0.03)).item()) < 1e-5
        assert not bool(valid[1, 1].item()), 'initializer filled entire map'
        saved = m.elevation_map.copy()
        mapper.last_initializer_attempt = -math.inf
        mapper.try_initialize_from_feet(stamp)
        assert bool(cp.array_equal(saved, m.elevation_map).item()), 'initializer ran repeatedly'
        response = mapper.handle_clear_map(Trigger.Request(), Trigger.Response())
        assert response.success and mapper.initializer_pending
        assert int(cp.count_nonzero(m.elevation_map[2]).item()) == 0
        mapper.use_initializer_at_start = False
        response = mapper.handle_clear_map(Trigger.Request(), Trigger.Response())
        assert response.success and not mapper.initializer_pending
        print(f'PASS GPU initializer: {count} valid cells, foot TF, world z, existing obstacle preserved, '
              'one-shot, missing TF, clear/reset, disabled mode')
    finally:
        observer.destroy_subscription(subscription)
        observer.destroy_node()
        adapter.destroy_node()
        mapper.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
