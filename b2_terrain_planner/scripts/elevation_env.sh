#!/usr/bin/env bash
# 맵 전용 환경. 제어권 전환·실기 실행은 수행하지 않는다.
set -euo pipefail
ELEVATION_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ELEVATION_PACKAGE_ROOT="$(cd "${ELEVATION_SCRIPT_DIR}/.." && pwd)"
if [[ -z "${ELEVATION_PREFIX:-}" ]]; then
    ELEVATION_PREFIX="${ELEVATION_PACKAGE_ROOT}/../.perception/install"
    # install-only 번들에서도 colcon 루트 overlay를 찾는다.
    ELEVATION_SEARCH="$ELEVATION_PACKAGE_ROOT"
    for _level in 1 2 3 4 5; do
        if [[ -f "$ELEVATION_SEARCH/_local_setup_util_sh.py" ]]; then
            ELEVATION_PREFIX="$ELEVATION_SEARCH"
            break
        fi
        ELEVATION_SEARCH="$(dirname "$ELEVATION_SEARCH")"
    done
fi
export ELEVATION_PREFIX
export ELEVATION_PYTHON="${ELEVATION_PYTHON:-${HOME}/miniforge3/envs/elevation_map/bin/python}"
export PYTHONNOUSERSITE=1
if [[ ! -x "$ELEVATION_PYTHON" ]]; then
    echo "[오류] ELEVATION_PYTHON에 CuPy/ROS 호환 Python을 지정하세요." >&2
    exit 2
fi
set +u
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
if [[ -f "$ELEVATION_PREFIX/local_setup.bash" ]]; then
    source "$ELEVATION_PREFIX/local_setup.bash"
fi
set -u
"$ELEVATION_PYTHON" - <<'PY'
import sys
from ament_index_python.packages import get_package_prefix
try:
    import numpy
    if int(numpy.__version__.split('.')[0]) >= 2:
        raise RuntimeError('ROS Humble cv_bridge requires NumPy 1.x; scripts/build.sh 참고')
    import rclpy, cv_bridge, tf_transformations, rosbag2_py
    import elevation_mapping_cupy, elevation_map_msgs, ros2_numpy
    import cupy
    for name in ['b2_terrain_planner', 'elevation_mapping_cupy', 'grid_map_msgs']:
        get_package_prefix(name)
    if cupy.cuda.runtime.getDeviceCount() < 1:
        raise RuntimeError('CUDA GPU not available')
    cupy.arange(4).sum().item()
except Exception as exc:
    print('[오류] elevation 의존성 검사 실패:', exc, file=sys.stderr)
    print('scripts/build.sh로 전용 overlay를 빌드하고 CUDA 드라이버를 확인하세요.',
          file=sys.stderr)
    sys.exit(2)
PY
