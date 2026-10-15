# B2 실기 / MuJoCo elevation map 실행

## 현재 입력 구성

- 실기: `/rslidar_points` + `/dog_odom` + 기존 센서 TF를 구독한다. 실기 제어
  실행파일 `run_real`이나 LowCmd publisher를 실행하지 않는다.
- 시뮬: `mujoco_node`의 LiDAR + GT `/odom` → 동일 입력 어댑터 → CuPy 맵.
  MuJoCo 월드 기준 몸통 pose를 사용하며 센서 odometry 추정 노드를 실행하지 않는다.
- 맵 4×4 m, 해상도 0.05 m, 정책 크롭 32×20(1.6×1.0 m), 발밑 보호 영역을 유지한다.
- 실기 센서 원천은 제공된 2026-10-06 B2 토픽 스냅샷을 따른다. 2026-10-08 사용자 확인으로
  실기 장착은 `../../b2_deploy_ws/src/pRbLibrary/model/unitree_b2/b2.urdf`의 `joint_lidar`와 동일하다.
  `/rslidar_points`는 LiDAR 좌표계라고 가정한다. 실제 메시지 프레임명·발행 주기와
  Helios 5515 모델명은 현장에서 확인하지 않았다.

## 빌드와 실행

`b2_terrain_planning/b2_terrain_planner`에서 실행한다. 기존 WBC 빌드/설치 디렉터리는 변경하지 않는다.

```bash
# 맵·상위 계획 전용 빌드
../scripts/build.sh

# MuJoCo도 필요한 경우 (b2_deploy_ws가 시뮬 실행파일을 소유)
../../b2_deploy_ws/scripts/build_elevation.sh --with-sim-runtime

# GT odometry + MuJoCo + elevation map + RViz
../../b2_deploy_ws/scripts/run_sim.sh --rviz

# 렌더링 없는 시뮬 / RViz 없는 맵
../../b2_deploy_ws/scripts/run_sim.sh --headless --elevation-map

# 이미 실행 중인 MuJoCo의 센서에 연결 (새 시뮬을 띄우지 않음)
./scripts/run_elevation_sim.sh --rviz

# 실기 센서만 구독하는 독립 맵
./scripts/run_elevation_real.sh --rviz
# 실제 프레임·장착값을 확인한 별도 YAML 사용
./scripts/run_elevation_real.sh --rviz --config /absolute/path/b2_sensor_input.yaml
```

`--xml`, `--model-dir`, `--headless`는 기존 시뮬 옵션과 함께 사용할 수 있다.
`--rviz`의 철자는 `--rivz`가 아니다. 기존 ROS launch 이름
`elevation_map_sim.launch.py`도 GT odometry 경로로 연결된다.

맵 산출물은 상위 저장소의 `.perception/build`, `.perception/install`에 저장한다.
MuJoCo 산출물은 `b2_deploy_ws/.perception/runtime_build`가 소유한다. 맵 실행 시 별도 빌드된 MuJoCo가 있으면
그 실행파일을 선택한다. `PKIRO_SIM_EXECUTABLE`로 명시적인 실행파일 선택도 가능하다.
일반 `run_sim.sh` 실행은 기존 설치본을 사용한다. 맵 없이 실행한 기존 시뮬에 연결할 때도
`/odom`과 `/lidar/points`가 발행되고 취득 시각이 보존되는 바이너리인지 확인한다.

맵 도구만 빌드하려면 `../scripts/build.sh`를 사용한다. 맵 도구에는 PCL 의존성이 없다. `ELEVATION_BUILD_ROOT`로
산출물 위치를 바꿨다면 실행 시 `ELEVATION_PREFIX=<산출물>/install`을 지정한다.
전용 MuJoCo 위치도 바뀐 경우 `PKIRO_SIM_EXECUTABLE`을 함께 지정한다.

CuPy 실행 Python은 기본 `~/miniforge3/envs/elevation_map/bin/python`이며
`ELEVATION_PYTHON`으로 변경할 수 있다. ROS launch는 시스템 Python, 맵은 지정한
Python으로 실행한다. `PYTHONNOUSERSITE=1`로 사용자 NumPy 2.x가 Humble/conda의
NumPy 1.x를 덮어쓰지 않게 한다. 맵 실행 전 CUDA 장치·커널·ROS 의존성을 확인한다.

실기는 기본 `ROS_DOMAIN_ID=0`, `ROS_LOCALHOST_ONLY=0`이다. `--interface`를 생략하면
설치된 CycloneDDS를 우선 사용하고, 없으면 안내를 출력하고 FastDDS를 사용한다.
이때 지정된 `RMW_IMPLEMENTATION`, `CYCLONEDDS_URI`, `ROS_LOCALHOST_ONLY`는 유지한다.
B2에서 조사에 사용한 CycloneDDS 설정 경로는
`/home/unitree/slam_config/cyclonedds_go2_B2_ws/cyclonedds.xml`이었다. 이 경로를
개발 PC에 임의 적용하지 않고, 해당 호스트의 네트워크 인터페이스에 맞춘 설정을 쓴다.
기존 검증 당시 개발 PC에는 FastDDS만 설치되어 있어 로컬 통합 테스트에서는
`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`를 명시했다. B2와의 실제 통신은 별도 확인 대상이다.

## 노트북에서 유선 LAN으로 실기 맵 실행

온보드는 기존 센서 노드와 WBC를 실행하고, 노트북은 입력 어댑터·CuPy 맵·RViz를
실행한다. 센서 토픽은 ROS 2 DDS로 직접 구독하며 relay와 추가 LiDAR 드라이버는 없다.
`deploy_real.sh`의 최소 Release 번들에는 노트북용 실행 프로그램이나 GPU 의존성을
추가하지 않는다. TAMOLS 실행·궤적 시각화는 별도 작업이며, 기존 맵 출력 계약을 사용한다.

### 1. 노트북 전용 빌드

Ubuntu 22.04 / ROS 2 Humble, NVIDIA GPU·CUDA·CuPy와 기존 맵 의존성이 준비된
노트북에서 실행한다. 아래 빌드는 상위 저장소의 `.perception`에 맵·상위 계획 도구를 설치한다.
`--with-sim-runtime`은 실기 토픽 수신에 필요하지 않다.

```bash
# CycloneDDS가 없는 노트북에서 한 번 설치
sudo apt install ros-humble-rmw-cyclonedds-cpp

# b2_terrain_planning/b2_terrain_planner에서
../scripts/build.sh
```

### 2. 유선 연결과 시간 확인

LAN 연결 후 노트북 NIC와 온보드 센서가 사용하는 NIC가 같은 서브넷인지 확인한다.
`192.168.123.164`는 배포 대상의 기본 IP일 뿐, 센서 발행 호스트·DDS 인터페이스와
같다고 단정하지 않는다. 아래 주소는 실제 연결에 맞춰 사용한다.

```bash
ip -brief address
ip route get 192.168.123.164
ping -c 3 192.168.123.164
```

IP 충돌이 없는 노트북 주소를 해당 유선 연결에 설정하고, DDS discovery용 멀티캐스트와
데이터용 UDP 통신이 두 호스트 사이에서 허용되는지 확인한다. ping 성공은 ROS 수신
성공을 의미하지 않는다. 온보드 센서 프로세스의 실제 `ROS_DOMAIN_ID`와 네트워크 설정을
확인한다. WBC 실행 셸의 설정만 확인해서 센서 프로세스도 같다고 가정하지 않는다.

두 호스트가 같은 시간 기준을 사용해야 한다. 각각 `timedatectl status`를 확인하고,
chrony 사용 환경이면 `chronyc tracking`, `chronyc sources -v`로 동기화 원천·오차를
확인한다. LAN 직결만으로 시계가 동기화되지는 않는다. 기존 NTP/chrony 구성으로
공통 시간 원천에 동기화한 뒤 메시지 stamp와 수신 시각을 비교한다. 시계 오차와
전송 지연은 `max_age=1.0 s`보다 충분히 작아야 한다. 이 문제를 입력 재타임스탬프나
허용 나이 완화로 우회하지 않는다. 실행 스크립트는 IP·방화벽·시간 설정을 변경하지 않는다.

### 3. 진단 터미널에서 원본 토픽 확인

아래 예시의 `enp3s0`는 실제 노트북 유선 NIC로, domain 0은 센서 발행 domain으로
바꾼다. NIC를 선택하는 XML은 맵 실행기의 `--interface`가 구성하는 것과 같다.
설정 항목은 [CycloneDDS 공식 문서](https://cyclonedds.io/docs/cyclonedds/0.10.5/config/cyclonedds_specifics.html)를 따른다.

```bash
source /opt/ros/humble/setup.bash
source ../.perception/install/local_setup.bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ELEVATION_NIC=enp3s0
export CYCLONEDDS_URI="<CycloneDDS xmlns=\"https://cdds.io/config\"><Domain Id=\"any\"><General><Interfaces><NetworkInterface name=\"${ELEVATION_NIC}\"/></Interfaces></General></Domain></CycloneDDS>"

# 이전 RMW/domain으로 실행된 로컬 CLI daemon이 남아 있으면 종료 후 재탐색
ros2 daemon stop
ros2 topic list -t
ros2 topic info /rslidar_points --verbose
ros2 topic info /dog_odom --verbose
ros2 topic echo /rslidar_points sensor_msgs/msg/PointCloud2 --once --field header --qos-reliability best_effort
ros2 topic echo /dog_odom nav_msgs/msg/Odometry --once --qos-reliability best_effort
ros2 topic hz /rslidar_points
ros2 topic hz /dog_odom
```

`hz`는 각각 측정 후 Ctrl+C로 종료한다. 점군 타입은 `PointCloud2`, odometry 타입은
`Odometry`인지 확인하고, endpoint QoS와 실제 메시지 수신을 함께 확인한다.
기존 어댑터는 sensor-data QoS로 구독한다. 목록에는 보여도 메시지가 오지 않으면
NIC·domain·QoS·UDP 통신을 점검한다. `hz` CLI의 결과만으로 맵 구독 상태를 단정하지 않는다.

점군 `header.frame_id`, odometry `header.frame_id`·`child_frame_id`, 각 stamp를
확인한다. 필요한 TF가 있으면 다음 명령으로 연결을 확인한다. 프레임 이름은 실제 값으로
바꾼다. 이 명령은 현재 TF 진단이며, 점군 취득 시각의 변환 유효성은 어댑터가 검사한다.

```bash
# 예: odometry child가 base_link가 아닐 때 필요한 연결
ros2 run tf2_ros tf2_echo ODOM_CHILD_FRAME base_link
# 기존 센서 TF를 사용하는 경우
ros2 run tf2_ros tf2_echo base_link CLOUD_FRAME
```

센서 TF가 없으면 기존 YAML의 확인된 URDF 장착값을 사용한다. 이 fallback은
odometry child에서 `base_link`까지 필요한 TF를 대신하지 않는다.

### 4. 맵과 RViz 실행

별도 터미널에서 저장소 루트로 이동해 실행한다. `--interface`는 CycloneDDS를 사용하며,
설치 여부·NIC 존재 여부를 검사한다. 다른 RMW 명시, 기존 `CYCLONEDDS_URI`,
`ROS_LOCALHOST_ONLY=1`이 충돌하면 원인을 출력하고 종료한다.

```bash
# 진단 셸의 사용자 XML 대신 --interface로 구성할 때 중복 설정 제거
unset CYCLONEDDS_URI
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
./scripts/run_elevation_real.sh --interface enp3s0 --rviz

# 실제 토픽·프레임 계약이 기본과 다르면 별도 입력 YAML 사용
./scripts/run_elevation_real.sh --interface enp3s0 --rviz \
  --config /absolute/path/b2_sensor_input.yaml
```

두 실행 명령 중 하나를 선택한다. 실행기는 적용한 domain·RMW·NIC·DDS 설정과
입력 YAML의 실제 경로를 출력한다. 환경 설정은 해당 프로세스와 자식 노드에만 적용되며,
다른 터미널에 전파되지 않는다. 사용자 DDS XML이 필요하면 진단 셸처럼
`CYCLONEDDS_URI`를 지정하고 `--interface`를 생략한다. 멀티캐스트 discovery를
사용할 수 없는 현장은 기존 사용자 XML에서 peer 설정 등을 관리한다.

앞서 구성한 진단 터미널에서 아래를 확인한다.

```bash
ros2 topic echo /elevation/input_status
ros2 topic hz /elevation_mapping_node/elevation_map_raw
```

각 명령을 별도로 실행해 입력 상태와 맵의 지속 갱신을 확인하고 RViz의 점군·높이 맵을
확인한다. 제어기를 실행하지 않는 센서 수신 시험에서 케이블을 분리하면 마지막 정상
입력 이후 기존 만료 처리에 따라 맵 발행이 중단되어야 한다. RViz는 마지막 화면을
보유할 수 있으므로 상태 토픽·새 맵 메시지 수신 여부로 판단한다. 재연결 후 신선한
점군·odometry와 필요한 TF가 확보되면 갱신이 재개되는지 확인한다.

## 입력 계약과 장애 처리

`config/elevation_input_real.yaml`이 실기 입력 계약을 소유한다.

| 설정 | 의미 |
|---|---|
| `cloud_topic` | 기본 `/rslidar_points` (`PointCloud2`) |
| `odom_topic` | 기본 `/dog_odom` (`Odometry`) |
| `body_frame` | 실기 기본 `base_link`; odometry child가 다르면 몸통까지 TF 필요 |
| `max_odom_gap` | 보간할 odometry 두 표본 사이 최대 간격, 기본 0.3 s |
| `max_age` | 입력 timestamp 최대 나이, 기본 1.0 s; 호스트 간 시간 동기화 필요 |
| `queue_wait` | odometry/TF 도착 대기, 기본 0.35 s |
| `extrinsic_parent/child` | TF가 없을 때 사용할 확인된 고정 변환의 프레임 이름 |
| `extrinsic_child_from_cloud` | 실기 기본 true: 센서 변환의 child를 점군 frame_id에서 읽음 |
| `extrinsic_xyz/xyzw` | 실기 기본 `[0.34218, 0, 0.17851]` m / `[0, 0, 0, 1]` |

기존 TF가 있으면 우선 사용하며, 없을 때만 사용자 확인을 받은 URDF 장착값을 사용한다.
`extrinsic_child_from_cloud`는 몸통→점군 변환에만 적용한다. odometry child가 `base_link`와
다를 때 필요한 몸통 변환을 대신하지 않는다. 점군 frame이 몸통과 같으면 추가 장착 변환을
적용하지 않는다. 고정 장착값 사용을 끄려면 `extrinsic_parent: ''`로 설정한다.

- 점군 시각으로 odometry를 보간하며 외삽하거나 현재 시각으로 재표기하지 않는다.
- XYZ FLOAT32/FLOAT64, endian, organized row padding을 해석한다. 추가 필드는 허용하고
  맵 입력은 유한 XYZ FLOAT32만 전달한다.
- odometry 부모 프레임부터 몸통 pose를 합성하고 몸통↔센서 변환을 확인한다.
  센서 좌표 자체는 유지하면서 동등한 변환을 전용 TF에 발행한다. 이름만 바꿔서
  다른 좌표인 것처럼 취급하지 않는다. 실기 장착값은 사용자 확인된 URDF 정의를 사용한다.
- 맵·RViz는 `/elevation/points`, `/elevation/tf`를 사용한다. 전용 프레임은
  `elevation_odom → elevation_base → elevation_lidar`이며 B2 기존 TF와 분리된다.
- 프레임 변경, 시간 역행, 표본 사이 1 m 또는 1.2 rad를 넘는 pose 변화는 원점 변경으로
  취급해 맵을 지운다. 이는 원점 변경 알림이 없는 실제 odometry에 대한 휴리스틱이며
  더 작은 재설정은 자동 판별할 수 없다. 해당 장치의 동작에 맞게 임계값을 조정한다.
- odometry/TF가 없거나 오래되면 해당 점군을 제외한다. 맵 실행기는 마지막 정상 입력
  후 1 s를 넘으면 raw map과 정책 크롭 발행도 중단한다. 기존 RViz 화면은 마지막
  데이터를 보유할 수 있으므로 `/elevation/input_status`를 함께 확인한다.
- CuPy 기본 코드의 최신 TF 대체 경로는 사용하지 않는다. 정확한 시각의 TF가 별도 DDS
  메시지보다 늦게 도착하면 최대 0.3 s 대기한다.

출력은 `/elevation_mapping_node/elevation_map_raw`,
`/elevation_mapping_node/policy_heightmap`, `/elevation_mapping_node/policy_mask`다.
크롭의 높이는 **지형 높이 − 몸통 높이**, 마스크는 실측 유효성이다. 무효 셀의 높이를
최근접 채워도 마스크를 유효로 바꾸지 않는다. 실행기는 CuPy 내부 높이가 center.z에
상대적인 것을 반영해 base.z가 중복 차감되지 않도록 보정한다.

## 시뮬 GT odometry의 의미와 한계

시뮬 입력은 `/lidar/points`와 `/odom`이다. `run_sim.sh --rviz`는 GT 발행을 활성화한다.
이미 실행 중인 시뮬에 맵만 연결할 때도 같은 토픽을 사용한다. ROS 시각은 system clock이며
LiDAR와 odometry는 센서 취득 시각을 보존한다. `/clock`은 사용하지 않는다.

| Odometry 필드 | GT 원천 / 기준 |
|---|---|
| `header.frame_id` | `odom`: MuJoCo 월드 좌표계와 동일한 원점·방향 |
| `child_frame_id` | `base_link`: 로봇 몸통 기준 |
| `pose.position` | free joint `qpos`의 월드 XYZ, 단위 m |
| `pose.orientation` | free joint `qpos`의 자세 quaternion, ROS 필드 순서 x/y/z/w |
| `twist.linear` | `qvel`의 월드 선속도를 몸통 좌표계로 회전, m/s |
| `twist.angular` | `qvel`의 몸통 각속도, rad/s |
| `header.stamp` | 해당 상태를 읽은 system-clock 시각 |
| pose/twist covariance | 0으로 발행; 실제 추정기의 불확실성을 모델링하지 않음 |

GT는 초기 위치·yaw를 임의로 0으로 재설정하지 않는다. 맵 어댑터는 점군 시각의 pose를
사용하며 twist는 맵에 전달하지 않는다. GT 경로 변경은 맵 입력에 한정되며 WBC/KF 입력을
교체하지 않는다. 기존 `odom → base_link → lidar_link` TF와 맵 전용 TF를 각각 발행한다.

이 구성은 지형 높이·맵 이동·정책 크롭을 검증한다. 실기 `/dog_odom`의 드리프트,
시간 동기화 오차, 제어 모드별 지속성과 원시 점군의 스캔 내부 모션 왜곡은 재현하지 않는다.
고정 베이스 모델은 기존 `GetBaseState`가 odometry를 제공하지 않으므로 이 경로의 대상이 아니다.

## 계획기 연결

맵 launch는 `terrain_planner_node`를 실행하지 않는다. 별도 실행 방법과 전체 흐름은
[루트 README](../README.md)의 워크플로우와 ROS 계획 실행 절차를 따른다.
맵의 프레임은 `elevation_odom`이고, 계획 노드의 기본 입력 프레임은 `odom`이다.
노드 내부 TF 변환은 없으므로 데이터의 원점·방향과 입력 `frame`을 맞춰야 한다.
마커의 프레임은 odometry header를 따르므로 RViz TF 연결도 별도로 확인한다.

입력 어댑터는 오래된 센서를 제외하고 맵 실행기는 오래된 입력의 맵 발행을 중단한다.
현재 계획 노드는 마지막 맵·odom을 저장하며 자체 만료 검사가 없다. 따라서 맵 입력이
끊겼을 때 계획 발행도 자동 중단된다고 해석하면 안 된다.

실기 수신·이동 중 맵 품질·WBC 추종은 별도 통합 확인 대상이다.
과거 시험 스크립트와 누적 실행 기록은 제거했으며, 변경 이력은 Git에서 관리한다.
