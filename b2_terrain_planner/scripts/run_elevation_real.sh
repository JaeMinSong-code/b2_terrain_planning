#!/usr/bin/env bash
# B2 센서 구독 전용 elevation map. run_real/LowCmd를 실행하지 않는다.
# 사용: ./run_elevation_real.sh [--interface <유선 NIC>] [--rviz] [--config <실기 입력 YAML>]
set -euo pipefail
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
    sed -n '2,3p' "$0"; exit 0
fi
MAP_RVIZ=false
MAP_CONFIG=""
MAP_INTERFACE=""
while [[ $# -gt 0 ]]; do
    case "$1" in
        --rviz) MAP_RVIZ=true; shift ;;
        --interface)
            [[ $# -ge 2 && -n "$2" && "$2" != --* ]] || {
                echo '유선 NIC 이름이 필요합니다.' >&2; exit 2;
            }
            MAP_INTERFACE="$2"; shift 2 ;;
        --config)
            [[ $# -ge 2 && "$2" != --* ]] || { echo '설정 경로가 필요합니다.' >&2; exit 2; }
            MAP_CONFIG="$(realpath "$2")"; shift 2 ;;
        *) echo "알 수 없는 옵션: $1" >&2; exit 2 ;;
    esac
done
if [[ -n "$MAP_INTERFACE" ]]; then
    # NIC 이름만 XML에 삽입한다. 경로 순회와 XML 특수문자는 허용하지 않는다.
    [[ "$MAP_INTERFACE" =~ ^[a-zA-Z0-9_.:-]+$ && -f "/sys/class/net/$MAP_INTERFACE/ifindex" ]] || {
        echo "[오류] 사용할 수 없는 네트워크 인터페이스: $MAP_INTERFACE" >&2; exit 2;
    }
    if [[ -n "${RMW_IMPLEMENTATION:-}" && "$RMW_IMPLEMENTATION" != rmw_cyclonedds_cpp ]]; then
        echo '[오류] --interface는 CycloneDDS 전용입니다. RMW_IMPLEMENTATION 설정을 확인하세요.' >&2
        exit 2
    fi
    if [[ -n "${CYCLONEDDS_URI:-}" ]]; then
        echo '[오류] --interface와 CYCLONEDDS_URI를 함께 지정할 수 없습니다. 하나만 사용하세요.' >&2
        exit 2
    fi
    if [[ "${ROS_LOCALHOST_ONLY:-0}" == 1 ]]; then
        echo '[오류] --interface 사용 시 ROS_LOCALHOST_ONLY=0이 필요합니다.' >&2
        exit 2
    fi
fi
source "$(dirname "${BASH_SOURCE[0]}")/elevation_env.sh"
if [[ -n "$MAP_INTERFACE" ]]; then
    if ! /usr/bin/python3 -c \
        'from ament_index_python.packages import get_package_prefix as p; p("rmw_cyclonedds_cpp")' \
        >/dev/null 2>&1; then
        echo '[오류] --interface에 필요한 CycloneDDS가 없습니다.' >&2
        echo '노트북에서 설치하세요: sudo apt install ros-humble-rmw-cyclonedds-cpp' >&2
        exit 2
    fi
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_URI="<CycloneDDS xmlns=\"https://cdds.io/config\"><Domain Id=\"any\"><General><Interfaces><NetworkInterface name=\"${MAP_INTERFACE}\"/></Interfaces></General></Domain></CycloneDDS>"
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
if [[ -z "$MAP_CONFIG" ]]; then
    MAP_CONFIG="$(/usr/bin/python3 -c \
        'from ament_index_python.packages import get_package_share_directory as p; print(p("b2_terrain_planner") + "/config/elevation_input_real.yaml")')"
fi
echo "[맵 네트워크] ROS_DOMAIN_ID=$ROS_DOMAIN_ID ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"
echo "[맵 네트워크] RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION interface=${MAP_INTERFACE:-DDS 설정에 따름}"
echo "[맵 네트워크] CYCLONEDDS_URI=${CYCLONEDDS_URI:-미설정}"
echo "[맵 입력] $MAP_CONFIG"
if [[ "$MAP_RVIZ" == true ]]; then
    "$ELEVATION_PYTHON" - <<'PY'
from ament_index_python.packages import get_package_prefix
get_package_prefix('rviz2')
get_package_prefix('grid_map_rviz_plugin')
PY
fi
MAP_LAUNCH_ARGS=(mode:=real start_sim:=false "launch_rviz:=$MAP_RVIZ" "input_config:=$MAP_CONFIG")
exec /usr/bin/python3 "$(command -v ros2)" launch b2_terrain_planner elevation_map.launch.py \
    "${MAP_LAUNCH_ARGS[@]}"
