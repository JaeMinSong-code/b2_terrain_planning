#!/usr/bin/env bash
# 맵·상위 계획 전용 overlay. --cpu-only는 ROS/GPU 없이 SQP만 빌드한다.
set -euo pipefail
TERRAIN_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${1:-}" == --help ]]; then
    echo '사용: scripts/build.sh [--cpu-only]'; exit 0
fi
if [[ "${1:-}" == --cpu-only ]]; then
    shift
    [[ $# == 0 ]] || { echo '지원하지 않는 인자' >&2; exit 2; }
    mkdir -p "$TERRAIN_ROOT/build"
    touch "$TERRAIN_ROOT/build/COLCON_IGNORE"
    cmake -S "$TERRAIN_ROOT/b2_terrain_planner" -B "$TERRAIN_ROOT/build" \
        -DB2_TERRAIN_STANDALONE=ON -DCMAKE_BUILD_TYPE=Release
    exec cmake --build "$TERRAIN_ROOT/build" -j "${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
fi
[[ $# == 0 ]] || { echo '지원하지 않는 인자' >&2; exit 2; }
TERRAIN_WS="${B2_WS_ROOT:-$(cd "$TERRAIN_ROOT/../.." && pwd)}"
TERRAIN_ARTIFACTS="${ELEVATION_BUILD_ROOT:-$TERRAIN_ROOT/.perception}"
mkdir -p "$TERRAIN_ARTIFACTS"
touch "$TERRAIN_ARTIFACTS/COLCON_IGNORE"
env -i HOME="$HOME" USER="${USER:-}" PATH=/usr/bin:/bin LANG=C.UTF-8 PYTHONNOUSERSITE=1 \
    CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-2}" \
    bash --noprofile --norc -c '
        set -eo pipefail
        source /opt/ros/humble/setup.bash
        colcon --log-base "$3/log" build \
            --build-base "$3/build" --install-base "$3/install" \
            --base-paths "$1/b2_terrain_planner" \
                "$2/src/elevation_mapping_cupy/elevation_map_msgs" \
                "$2/src/elevation_mapping_cupy/elevation_mapping_cupy" \
                "$2/src/ros2_numpy" \
            --packages-select b2_terrain_planner elevation_map_msgs elevation_mapping_cupy ros2_numpy \
            --executor sequential --symlink-install \
            --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
                -DPython3_EXECUTABLE=/usr/bin/python3
    ' bash "$TERRAIN_ROOT" "$TERRAIN_WS" "$TERRAIN_ARTIFACTS"
