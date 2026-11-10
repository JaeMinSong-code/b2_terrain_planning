"""GPU 출력의 축·NaN·높이와 전달 경로의 발행 의미를 검증한다.

B2ElevationMap(공식 CuPy 원본을 수정하지 않는 서브클래스)의 raw 레이어 일괄 복사와
발행 인코딩을 확인한다. cupy가 필요하므로 elevation conda 환경에서 실행한다. 예:

  ELEVATION_PYTHON=~/miniforge3/envs/elevation_map/bin/python
  "$ELEVATION_PYTHON" -m pytest \\
    src/b2_terrain_planning/b2_terrain_planner/development/tests/map_publication_test.py
"""
import os
import sys
import threading
from unittest.mock import patch
import cupy as cp
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'scripts'))
from b2_elevation_map import B2ElevationMap
from elevation_mapping_cupy.gridmap_utils import encode_layer_to_multiarray, decode_multiarray_to_rows_cols
from rclpy.serialization import serialize_message, deserialize_message
from std_msgs.msg import Float32MultiArray


def make_export_map():
    m = B2ElevationMap.__new__(B2ElevationMap)
    m.map_lock = threading.Lock()
    m.cell_n = 8
    m.elevation_map = cp.arange(7 * 8 * 8, dtype=cp.float32).reshape(7, 8, 8) / 13
    m.elevation_map[2] = cp.indices((8, 8)).sum(axis=0) % 2
    m.center = cp.asarray([1.25, -2.5, 3.75], dtype=cp.float32)
    return m


def test_raw_batch_matches_single_layers_with_one_transfer():
    m = make_export_map()
    names = ['variance', 'elevation', 'is_valid', 'elevation']
    expected = np.empty((4, 6, 6), dtype=np.float32)
    for name, target in zip(names, expected):
        m.get_map_with_name_ref(name, target)
    result = np.empty_like(expected)
    with patch.object(cp, 'asnumpy', wraps=cp.asnumpy) as transfer:
        m.get_maps_with_names_ref(names, result)
        assert transfer.call_count == 1
    np.testing.assert_array_equal(result, expected)
    height = cp.asnumpy(m.elevation_map[0, 1:-1, 1:-1]) + 3.75
    valid = cp.asnumpy(m.elevation_map[2, 1:-1, 1:-1]) > .5
    np.testing.assert_array_equal(result[1], np.where(valid, height, np.nan).T[::-1, ::-1])


def test_nonraw_layers_keep_legacy_export():
    m = make_export_map()
    result = np.empty((2, 6, 6), dtype=np.float32)
    expected = np.empty_like(result)
    for name, target in zip(['time', 'elevation'], expected):
        m.get_map_with_name_ref(name, target)
    m.get_maps_with_names_ref(['time', 'elevation'], result)
    np.testing.assert_array_equal(result, expected)


def test_encoding_owns_buffer_and_survives_ros_serialization():
    source = np.arange(30, dtype=np.float32).reshape(5, 6)[:, ::-1]
    source[1, 2] = np.nan
    expected = source.copy()
    for layout in ['gridmap_column', 'row_major']:
        msg = encode_layer_to_multiarray(source, layout)
        wire = serialize_message(msg)
        decoded = deserialize_message(wire, Float32MultiArray)
        np.testing.assert_array_equal(decode_multiarray_to_rows_cols('height', decoded), expected)
    msg = encode_layer_to_multiarray(source)
    source[:] = -100
    np.testing.assert_array_equal(decode_multiarray_to_rows_cols('height', msg), expected)
