# B2 지형 맵·상위 계획

B2 센서 또는 MuJoCo의 점군·odometry로 GPU 높이맵을 만들고, C++ TAMOLS 계획기로 몸통
CoM 궤적과 착지점을 계산한다. ROS 노드는 계획과 RViz 마커를 발행하고, `tamols_solve`
CLI는 B2 상태 YAML을 읽어 독립적으로 계획을 계산한다.

이 저장소의 범위는 **센서 입력 → 지형 맵 → 상위 계획 → ROS 출력**이다. WBC·모터 제어·
MuJoCo 실행파일은 별도 `b2_deploy_ws`가 소유한다. 이 저장소에는 pRbLibrary 소스나
서브모듈이 없으며 B2 URDF 복제본만 포함한다.

알고리즘 근거, 구현 경계, 진단 토픽, 맵 런타임 절차, 평지 검증, 검증 기록 등 자세한
내용은 **[docs/design_notes.md](docs/design_notes.md)** 하나에 모아 두었다.

## 의존성

사람이 직접 준비해야 하는 의존성이다.

**CPU 계획기 (ROS·GPU 불필요)**
- CMake, C++17 컴파일러, `pkg-config`
- Eigen3 (≥3.3), yaml-cpp, LAPACK(LP64), tinyxml2, RBDL + URDF reader
- OSQP **0.6.x** double-precision C API (PROFILING 포함) — OSQP 1.x는 C API가 달라 비호환
- 설정 생성용 Python 3 + PyYAML (`python3-yaml`)
- 검증 기준 버전: OSQP 0.6.3, RBDL 3.3.1, tinyxml2 9.0.0

**맵·ROS 실행 (추가)**
- ROS 2 Humble, 같은 워크스페이스의 `elevation_mapping_cupy`·`ros2_numpy`
- NVIDIA GPU·CUDA·CuPy와 해당 Python 환경, NumPy **1.x** (Humble `cv_bridge` 요구)
- 실기 유선 LAN 수신에는 선택적으로 `ros-humble-rmw-cyclonedds-cpp`

## 빌드

```bash
# CPU 계획기만 빌드 (저장소 루트에서)
cmake -S b2_terrain_planner -B /tmp/b2-terrain-cpu-build \
  -DB2_TERRAIN_STANDALONE=ON -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
cmake --build /tmp/b2-terrain-cpu-build -j2
```

OSQP 위치는 `OSQP_INCLUDE_DIR`·`OSQP_LIBRARY`, RBDL 위치는 `RBDL_DIR` 또는
`CUSTOM_RBDL_PATH`로 지정할 수 있다.

```bash
# 맵·ROS 상위 계획은 워크스페이스에서 colcon으로 빌드
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --base-paths src \
  --packages-up-to b2_terrain_planner elevation_map_msgs elevation_mapping_cupy ros2_numpy \
  --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DPython3_EXECUTABLE=/usr/bin/python3
ctest --test-dir build/b2_terrain_planner --output-on-failure
```

맵 실행은 기본으로 워크스페이스 설치본을 찾는다. 별도 설치본을 쓰려면
`ELEVATION_PREFIX=<설치 경로>`를, CuPy 실행 Python을 바꾸려면 `ELEVATION_PYTHON=<python>`을
지정한다(기본 `~/miniforge3/envs/elevation_map/bin/python`).

## 사용

### 1. B2 YAML 계획 (독립 CLI)

```bash
/tmp/b2-terrain-cpu-build/tamols_solve --input b2_terrain_planner/config/b2_terrain.yaml
/tmp/b2-terrain-cpu-build/tamols_solve --speed 0.1 --fast-replanning
/tmp/b2-terrain-cpu-build/tamols_solve --help
```

### 2. elevation map 실행

맵 실행기는 환경변수로 설정하는 단일 스크립트다. 입력 어댑터·맵·선택적 RViz를 실행하며
`terrain_planner_node`는 띄우지 않는다.

```bash
# 이미 실행 중인 MuJoCo 센서에 맵과 RViz 연결
RVIZ=1 ./b2_terrain_planner/scripts/run_elevation.sh

# 실기 센서에 맵과 RViz 연결 (NIC 이름은 실제 환경에 맞춘다)
MODE=real INTERFACE=enp3s0 RVIZ=1 ./b2_terrain_planner/scripts/run_elevation.sh
```

| 환경변수 | 기본값 | 용도 |
| --- | --- | --- |
| `MODE` | `sim` | `sim`=MuJoCo GT `/odom`+LiDAR, `real`=B2 센서 구독 |
| `RVIZ` | `0` | `1`이면 RViz를 함께 실행 |
| `START_SIM` | `0` | sim 모드에서 `1`이면 `b2_deploy` mujoco_node도 실행(GT 진단용) |
| `INTERFACE` | (없음) | real 모드 유선 NIC. CycloneDDS 전용 설정을 구성 |
| `INPUT_CONFIG` | (모드 기본) | 입력 ROS 파라미터 YAML 직접 지정 |
| `ELEVATION_PYTHON` | miniforge `elevation_map` | CuPy·ROS 호환 Python |
| `ELEVATION_PREFIX` | 워크스페이스 install | 맵 패키지 설치 경로 |

`ROS_DOMAIN_ID`·`ROS_LOCALHOST_ONLY`·`RMW_IMPLEMENTATION`·`CYCLONEDDS_URI`는 지정하면
그대로 유지한다. 유선 LAN 실기 맵 절차와 입력 계약은 design_notes의
[맵 입력·런타임 절차](docs/design_notes.md#맵-입력런타임-절차)를 따른다.

### 3. 계획 노드 실행

별도 터미널에서 실행한다. 시뮬·실기 모두 WBC 실행 노드의 `/robot/locomotion_state`를
구독하므로 WBC task로 추정 상태가 유효해져야 계획을 시작한다. 맵·상태의 좌표 원점·방향이
같아야 하며(노드 내부 TF 변환 없음), 맵 입력 odometry는 `/kf/odom`으로 맞춘다.

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/local_setup.bash
ros2 run b2_terrain_planner terrain_planner_node --ros-args \
  -p state_topic:=/robot/locomotion_state -p state_frame:=odom -p frame:=elevation_odom
```

| ROS 노드 설정 | 기본값 |
|---|---|
| `state_topic`, `state_frame` | `/robot/locomotion_state`, `odom` |
| `state_max_age`, `map_max_age` | 0.25 s, 1.0 s |
| `map_topic` | `/elevation_mapping_node/elevation_map_raw` |
| `frame` | `odom` — 입력 맵은 기본 `elevation_odom`이므로 실행 시 확인 |
| `command` | `[0.1, 0.0, 0.0]` |
| `plan_rate_hz`, `time_limit` | 10 Hz, 0.1 s |
| `sample_dt`, `swing_height` | 0.02 s, 0.10 m |
| `require_map`, `publish_markers` | true, true |
| `plan_topic`, `marker_topic` | `/terrain_planner/plan`, `/terrain_planner/markers` |

## 설정

설정 원본은 `b2_terrain_planner/config/b2_terrain.yaml` 하나다. 빌드 시
`scripts/generate_config.py`가 기존 이름의 ROS 파라미터 YAML과 CLI 예제 YAML을 설치
경로에 생성한다. 생성 파일을 직접 수정하지 말고 원본을 고친 뒤 사용하는 설치 경로를 다시
빌드한다. RViz 화면은 `config/*.rviz`에서 관리한다. 필드 설명은 design_notes를 참고한다.

## 저장소 구조

```text
b2_terrain_planning/
├── b2_terrain_planner/
│   ├── src/, include/    # SQP, 지형, RBDL 모델, CLI, ROS 노드
│   ├── development/
│   │   ├── codegen/      # CasADi 수식과 C 미분 커널 생성기
│   │   ├── generated/    # 일반 빌드에서 사용하는 생성 C 코드
│   │   └── tests/        # 계약·회귀·ROS 통합 검증 소스
│   ├── model/            # B2 URDF와 출처
│   ├── msg/              # LocomotionPlan·LocomotionState 메시지
│   ├── config/           # 센서·맵·RViz·B2 상태 설정
│   └── scripts/, launch/ # 맵 실행기(run_elevation.sh)와 launch
├── AGENTS.md             # 이 저장소의 단일 작업 규칙
└── docs/
    ├── design_notes.md   # 상세 설계·운영·검증 노트 (AI 참고)
    └── source_manifest.json  # 외부 맵 저장소·패치 기준
```

B2 모델의 기준 커밋·해시는 [모델 출처](b2_terrain_planner/model/README.md), 외부 맵
저장소의 기준은 [source_manifest.json](docs/source_manifest.json)에 기록한다. 커널 생성기는
CasADi 3.7.2·NumPy가 있는 환경에서 `development/codegen/generate_kernels.py`로 실행한다
(일반 빌드는 이미 포함된 `development/generated/`를 사용한다).
