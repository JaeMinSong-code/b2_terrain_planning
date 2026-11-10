"""GridMap geometry must follow the stored grid, including between cell shifts.

B2 발행 로직은 StrictElevationMappingNode(공식 CuPy 노드를 수정하지 않는 서브클래스)에 있다.
import가 cupy를 당기므로 elevation conda 환경에서 실행한다. 예:

  ELEVATION_PYTHON=~/miniforge3/envs/elevation_map/bin/python
  "$ELEVATION_PYTHON" -m pytest \\
    src/b2_terrain_planning/b2_terrain_planner/development/tests/map_center_publication_test.py
"""
import os
import sys
import time
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Quaternion, Vector3

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'scripts'))
from elevation_mapping_runner import StrictElevationMappingNode
from elevation_mapping_cupy.gridmap_utils import encode_layer_to_multiarray


@pytest.mark.parametrize("robot_xy", [(1.01, -0.99), (1.024, -1.024),
                                      (1.076, -0.926), (0.924, -1.076)])
def test_publication_uses_grid_center_instead_of_continuous_robot_pose(robot_xy):
    resolution = 0.05
    center = np.array([*robot_xy, 0.3], dtype=np.float32)
    center[:2] = np.round(center[:2] / resolution) * resolution
    grid = Mock(resolution=resolution, cell_n=6)
    grid.get_center_position.side_effect = lambda result: np.copyto(result[0], center)
    grid.get_map_with_name_ref.side_effect = lambda name, result: result.fill(0.2)
    grid.get_maps_with_names_ref.side_effect = lambda names, result: result.fill(0.2)
    publisher = Mock()
    publisher.get_subscription_count.return_value = 1
    node = SimpleNamespace(
        _map=grid,
        _map_q=Quaternion(w=1.0),
        _map_t=Vector3(x=robot_xy[0], y=robot_xy[1], z=0.3),
        _publishers_dict={"raw": publisher},
        _last_t=Time(sec=42),
        map_frame="elevation_odom",
        my_publishers={"raw": {"layers": ["elevation"],
                               "basic_layers": ["elevation"]}},
        _map_layer_buffers={},
        _map_data=np.empty((4, 4), dtype=np.float32),
        _numpy_to_multiarray=encode_layer_to_multiarray,
        last_accepted=time.monotonic(),
    )
    node._get_map_center = lambda: StrictElevationMappingNode._get_map_center(node)
    StrictElevationMappingNode.publish_map(node, "raw")
    msg = publisher.publish.call_args.args[0]
    np.testing.assert_array_equal(
        [msg.info.pose.position.x, msg.info.pose.position.y], center[:2])
    assert msg.info.pose.position.z == 0.0
    assert msg.info.pose.orientation == Quaternion(w=1.0)
    assert msg.info.resolution == resolution
    assert msg.info.length_x == msg.info.length_y == 4 * resolution
    assert msg.header.stamp == node._last_t
    assert msg.header.frame_id == node.map_frame
    assert msg.layers == ["elevation"]
    assert len(msg.data[0].data) == 16
