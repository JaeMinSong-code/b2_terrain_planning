#!/usr/bin/env bash
# elevation map 실행기 (sim/real 공용). 환경변수로 모드와 옵션을 선택한다.
# 제어권 전환이나 실기 제어(run_real/LowCmd)는 실행하지 않는다.
#
# 환경변수:
#   MODE=sim|real       기본 sim. sim은 MuJoCo GT /odom+LiDAR, real은 B2 센서 구독.
#   RVIZ=0|1            기본 0. 1이면 RViz를 함께 띄운다.
#   START_SIM=0|1       기본 0. sim 모드에서 1이면 b2_deploy mujoco_node도 실행(GT 진단용).
#   INTERFACE=<NIC>     real 모드 유선 NIC. CycloneDDS 전용 설정을 구성한다.
#   INPUT_CONFIG=<파일>  입력 ROS 파라미터 YAML 직접 지정(생략 시 모드 기본값).
#   ROS_DOMAIN_ID, ROS_LOCALHOST_ONLY, RMW_IMPLEMENTATION, CYCLONEDDS_URI: 지정 시 유지한다.
#   ELEVATION_PYTHON, ELEVATION_PREFIX: elevation_env.sh 참고.
#
# 예) 시뮬+RViz:        RVIZ=1 ./run_elevation.sh
#     실기+RViz+NIC:    MODE=real INTERFACE=enp3s0 RVIZ=1 ./run_elevation.sh
set -euo pipefail
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
    sed -n '2,16p' "$0"; exit 0
fi
if [[ $# -gt 0 ]]; then
    echo "[오류] 이 스크립트는 환경변수로만 설정합니다(--help 참고). 알 수 없는 인자: $*" >&2
    exit 2
fi

MODE="${MODE:-sim}"
RVIZ="${RVIZ:-0}"
START_SIM="${START_SIM:-0}"
INTERFACE="${INTERFACE:-}"
INPUT_CONFIG="${INPUT_CONFIG:-}"

on() { [[ "$1" == 1 || "$1" == true || "$1" == yes ]]; }

if [[ "$MODE" != sim && "$MODE" != real ]]; then
    echo "[오류] MODE는 sim 또는 real이어야 합니다: $MODE" >&2
    exit 2
fi
if [[ -n "$INTERFACE" && "$MODE" != real ]]; then
    echo "[오류] INTERFACE는 real 모드 전용입니다. MODE=real로 실행하세요." >&2
    exit 2
fi
if on "$START_SIM" && [[ "$MODE" != sim ]]; then
    echo "[오류] START_SIM은 sim 모드 전용입니다." >&2
    exit 2
fi

LAUNCH_RVIZ=false
if on "$RVIZ"; then LAUNCH_RVIZ=true; fi
LAUNCH_START_SIM=false
if on "$START_SIM"; then LAUNCH_START_SIM=true; fi

# real 모드 유선 NIC는 CycloneDDS 전용이다. 경로 순회·XML 특수문자는 허용하지 않는다.
if [[ -n "$INTERFACE" ]]; then
    [[ "$INTERFACE" =~ ^[a-zA-Z0-9_.:-]+$ && -f "/sys/class/net/$INTERFACE/ifindex" ]] || {
        echo "[오류] 사용할 수 없는 네트워크 인터페이스: $INTERFACE" >&2; exit 2;
    }
    if [[ -n "${RMW_IMPLEMENTATION:-}" && "$RMW_IMPLEMENTATION" != rmw_cyclonedds_cpp ]]; then
        echo '[오류] INTERFACE는 CycloneDDS 전용입니다. RMW_IMPLEMENTATION 설정을 확인하세요.' >&2
        exit 2
    fi
    if [[ -n "${CYCLONEDDS_URI:-}" ]]; then
        echo '[오류] INTERFACE와 CYCLONEDDS_URI를 함께 지정할 수 없습니다. 하나만 사용하세요.' >&2
        exit 2
    fi
    if [[ "${ROS_LOCALHOST_ONLY:-0}" == 1 ]]; then
        echo '[오류] INTERFACE 사용 시 ROS_LOCALHOST_ONLY=0이 필요합니다.' >&2
        exit 2
    fi
fi
if [[ -n "$INPUT_CONFIG" ]]; then
    INPUT_CONFIG="$(realpath "$INPUT_CONFIG")"
fi

source "$(dirname "${BASH_SOURCE[0]}")/elevation_env.sh"

# real 모드 네트워크(RMW/도메인/DDS) 구성은 sim에서는 건너뛴다.
if [[ "$MODE" == real ]]; then
    if [[ -n "$INTERFACE" ]]; then
        if ! /usr/bin/python3 -c \
            'from ament_index_python.packages import get_package_prefix as p; p("rmw_cyclonedds_cpp")' \
            >/dev/null 2>&1; then
            echo '[오류] INTERFACE에 필요한 CycloneDDS가 없습니다.' >&2
            echo '노트북에서 설치하세요: sudo apt install ros-humble-rmw-cyclonedds-cpp' >&2
            exit 2
        fi
        export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
        export CYCLONEDDS_URI="<CycloneDDS xmlns=\"https://cdds.io/config\"><Domain Id=\"any\"><General><Interfaces><NetworkInterface name=\"${INTERFACE}\"/></Interfaces></General></Domain></CycloneDDS>"
    elif [[ -z "${RMW_IMPLEMENTATION:-}" ]]; then
        if /usr/bin/python3 -c \
            'from ament_index_python.packages import get_package_prefix as p; p("rmw_cyclonedds_cpp")' \
            >/dev/null 2>&1; then
            export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
        else
            export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
            echo '[정보] CycloneDDS 미설치: FastDDS 사용. 실기 네트워크 수신은 별도 확인하세요.'
        fi
    fi
    export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
    export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
    echo "[맵 네트워크] ROS_DOMAIN_ID=$ROS_DOMAIN_ID ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"
    echo "[맵 네트워크] RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION interface=${INTERFACE:-DDS 설정에 따름}"
    echo "[맵 네트워크] CYCLONEDDS_URI=${CYCLONEDDS_URI:-미설정}"
    echo "[맵 입력] ${INPUT_CONFIG:-모드 기본값}"
fi

if on "$RVIZ"; then
    "$ELEVATION_PYTHON" - <<'PY'
from ament_index_python.packages import get_package_prefix
get_package_prefix('rviz2')
get_package_prefix('grid_map_rviz_plugin')
PY
fi

LAUNCH_ARGS=("mode:=$MODE" "start_sim:=$LAUNCH_START_SIM" "launch_rviz:=$LAUNCH_RVIZ")
if [[ -n "$INPUT_CONFIG" ]]; then
    LAUNCH_ARGS+=("input_config:=$INPUT_CONFIG")
fi
exec /usr/bin/python3 "$(command -v ros2)" launch b2_terrain_planner elevation_map.launch.py \
    "${LAUNCH_ARGS[@]}"
