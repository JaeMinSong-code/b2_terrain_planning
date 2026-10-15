#!/usr/bin/env bash
# 이미 실행 중인 MuJoCo의 GT /odom + LiDAR에 맵을 연결한다.
# 사용: ./run_elevation_sim.sh [--rviz] [--start-sim]
set -euo pipefail
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
    sed -n '2,3p' "$0"; exit 0
fi
MAP_RVIZ=false
MAP_START_SIM=false
while [[ $# -gt 0 ]]; do
    case "$1" in
        --rviz) MAP_RVIZ=true; shift ;;
        --start-sim) MAP_START_SIM=true; shift ;;
        *) echo "알 수 없는 옵션: $1" >&2; exit 2 ;;
    esac
done
source "$(dirname "${BASH_SOURCE[0]}")/elevation_env.sh"
if [[ "$MAP_RVIZ" == true ]]; then
    "$ELEVATION_PYTHON" - <<'PY'
from ament_index_python.packages import get_package_prefix
get_package_prefix('rviz2')
get_package_prefix('grid_map_rviz_plugin')
PY
fi
exec /usr/bin/python3 "$(command -v ros2)" launch b2_terrain_planner elevation_map.launch.py \
    mode:=sim start_sim:="$MAP_START_SIM" launch_rviz:="$MAP_RVIZ"
