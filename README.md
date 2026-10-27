# B2 지형 맵·상위 계획

WBC와 함께 사용하는 통합 실행은 `b2_deploy_ws/scripts/run_laptop.sh`가 담당한다.
GUI·RViz·지도·TAMOLS·시각화 TF를 함께 실행하고, 지도 입력은 `/kf/odom`으로 맞춘다.
로봇 측은 `b2_deploy_ws/scripts/run_robot.sh`로 실행한다(개발 PC 기본 MuJoCo).
자세한 사용법은 [두 역할 실행 문서](../b2_deploy_ws/docs/two_process_runtime.md)를 따른다.
통합 설정에서는 로봇이 제어권을 검증해 발행한 `/robot/locomotion_command`
(`geometry_msgs/TwistStamped`)를 속도 목표로 사용한다. `base_heading`의 수평 전후/좌우
속도와 yaw 속도이며, 기본 0.25초 동안 새 명령이 없으면 0으로 처리한다.
`/terrain_planner_node/effective_command`에서 실제 계획 입력에 사용하는 값을 확인할 수 있다.
단독 실행의 고정 `command` 파라미터는 `command_topic`이 비어 있을 때만 사용한다.
통합 설정은 현재 접촉 발 주변에 발 링크 높이와 같은 국소 지면을 가정하는 임시 보정을 켠다.
명령 전달 및 국소 지도 보정과 전체 폐루프 보행 성공은 별도로 검증한다.
`/terrain_planner/elevation_map_corrected`는 실제 SQP 입력의 z 보정·접촉발 패치 적용 후
지도를 최대 10 Hz로 발행한다(reliable/transient-local, 마지막 1개). 원본 지도 시각과
좌표를 유지하며 미관측 셀은 NaN이다. 새 계획 입력이 없으면 마지막 지도만 유지한다.
비평지 sim `--tamols --rviz`의 기본 표시는 이 지도이며, 원본은 `RawElevationMap`을 켜서 본다.
sim `--tamols`는 원본·보정 지도와 발 상태·odom 전체 메시지를 deploy의
`runtime_logs/map_history_*` rosbag으로 기록한다. 원본 지도 보정이나 보행 개선 자체를 뜻하지 않는다.
아래의 GT/실기 센서 전용 맵 실행은 개별 진단용 경로다.

평지 알고리즘 검증은 두 실행기에 모두 `--flat`을 붙인다. 별도 평면 MuJoCo XML과
기존 LiDAR/CuPy 지도를 사용하고, 입력 GridMap의 구조를 유지한 채 높이·분산·유효성만
평탄화한다. 평면 높이는 최근 유효 센서 관측을 따라 갱신해 odometry 드리프트를 반영한다.
초기 발 링크/지면 높이 차이만 고정 오프셋으로 유지한다. TAMOLS와 RViz는 `/terrain_planner/elevation_map_flat`을 받는다.
현재 지지발은 추정 위치를 그대로 사용하며 지도와 5mm 이상 높이가 달라도 입력을 거부하지 않는다.
현재 발·확정 착지점의 지도 조회를 재계획 조건으로 요구하지 않는다. 새 착지점의 지도 조회와
SQP의 지형 비용·GIAC·마찰·도달 길이 제약은 유지한다.
최적화 뒤 별도로 수행하던 지도 높이·관절 IK 검사는 제거했다.
이 변경은 높이 오차를 보정하거나 계산 성공을 보장하는 기능이 아니다. 계산 시간 제한과
접촉·궤적 연결을 포함한 연속 보행 검증은 별도다.
실행 순서, 재현 명령과 확인된 한계는 [평지 검증 문서](docs/flat_validation.md)를 따른다.

B2 센서 또는 MuJoCo의 점군·odometry로 GPU 높이맵을 만들고, C++ TAMOLS 계획기로
몸통 CoM 궤적과 착지점을 계산한다. ROS 노드는 계획과 RViz 마커를 발행하고,
`tamols_solve`는 B2 상태 YAML을 읽어 독립적으로 계획을 계산한다.

이 저장소의 범위는 **센서 입력 → 지형 맵 → 상위 계획 → ROS 출력**이다.
WBC·모터 제어·MuJoCo 실행파일은 별도 `b2_deploy_ws`가 소유한다.
이 저장소에는 pRbLibrary 소스나 서브모듈이 없으며, B2 URDF 복제본만 포함한다.

## 시작 시 CuPy 발밑 초기화

통합 elevation map 설정은 `use_initializer_at_start: true`, `initialize_method: linear`,
`initialize_frame_id: [FL_foot, FR_foot, RL_foot, RR_foot]`,
`initialize_tf_offset: [0.0, 0.0, 0.0, 0.0]`, `dilation_size_initialize: 2`를 사용한다.
adapter가 `/robot/locomotion_state`의 발 위치를 `/elevation/tf`로 전달하므로 RViz나
robot_state_publisher가 없어도 된다. 입력 odom과 발 상태의 좌표계가 같아야 한다.

runner는 네 발 TF가 준비되면 CuPy의 기존 선형 보간과 dilation으로 미관측 영역을
한 번 초기화한다. 이미 관측된 셀은 덮어쓰지 않으며 발 TF를 기다리는 중에도 LiDAR
융합은 계속한다. 로그 `Foot initializer complete: ... filled=N cells`로 실행을 확인한다.
map clear 후에는 다시 초기화한다. 주기적인 평탄화나 스윙마다 새 평면을 만드는 기능은 아니다.
초기화 셀은 실제 CuPy 원본 지도에 들어가므로 raw·보정 지도와 rosbag 양쪽에서 확인된다.
설정 파일은 `b2_terrain_planner/config/elevation_map_sim.yaml`이며 false로 끌 수 있다.

## 접촉 발의 작은 영역과 높이 기억

`terrain_planner_wbc.yaml`의 `support_patch.enabled: true`이면,
`contact_confirmed && contact_estimated`인 발 주변 셀을 발의 Z 높이로 기억한다.
기본 `support_patch.half_width: 0.05`는 X/Y 각 방향 5 cm(10×10 cm) 영역이다.
필터·미분·보간을 위해 영역을 자동으로 넓히지 않는다. 해당 영역 안의 셀 중심만
갱신하므로 실제 표시 경계는 지도 해상도에 따른다. 겹치면 가장 가까운 접촉 발을 따른다.

기억한 셀은 발을 들어도 유지하며, 새 원본 지도가 도착해도 지우지 않는다.
같은 셀에 다시 접촉하면 최신 발 높이로 갱신한다. 지도 이동 시 동일 좌표의 셀을 유지하고
로컬 지도 밖의 기억은 버린다. 지도 frame·해상도 변경, ROS 시각 역행, 노드 재시작 시
기억을 초기화한다. 기억은 현재 로컬 지도 크기로 제한하며 파일에서 복원하지 않는다.

발 링크 높이를 지면으로 취급하는 기존 가정은 유지한다(발 두께 보정 아님).
보정 셀의 유효성은 1, 분산은 0이다. 원본 관측 지도에는 쓰지 않고, datum 오프셋을 적용한
계획 입력 복사본 위에 접촉 기억을 덧씌운다. 따라서 발을 뗀 뒤에도 같은 셀이
TAMOLS와 RViz의 `CorrectedElevationMap`에 남는다. 그 밖의 관측 지형·미관측 영역은 보존한다.
이는 시간에 따른 odom 드리프트나 원점 재설정을 추정해 보상하는 기능은 아니다.

단독 ROS 실행은 기본 OFF이며 통합 설정도 `support_patch.enabled: false`로 끌 수 있다.
계획 입력 갱신이 멈추면 보정 지도 표시는 마지막 입력을 유지한다.
검증은 좁은 영역, 이륙 후 보존, 재착지, 로컬 지도 이동·초기화에 관한 것이며
최적화 성공률이나 폐루프 보행 성공을 보장하지 않는다.

## 전체 워크플로우

```mermaid
flowchart TD
    Real["실기 센서<br/>/rslidar_points · /dog_odom"] --> Adapter
    Sim["MuJoCo<br/>/lidar/points · /odom"] --> Adapter
    Adapter["elevation_input_adapter.py<br/>시각 정렬 · 좌표 변환 · 입력 만료 처리"] --> Points["/elevation/points<br/>/elevation/tf"]
    Points --> Map["elevation_mapping_runner.py<br/>elevation_mapping_cupy · GPU 높이맵"]
    Map --> Grid["/elevation_mapping_node/elevation_map_raw"]
    Map --> Crop["policy_heightmap · policy_mask"]
    WBC -->|/robot/locomotion_state| Node
    WBC -->|/robot/locomotion_command| Node
    Grid --> Node["terrain_planner_node<br/>별도 실행 · WBC CoM/발 상태 수신"]
    Model["B2 URDF · 명목 자세 · 속도 명령"] --> Node
    Node --> Solve["C++ Solve<br/>지형 처리 · 보행 구간 · 초기 궤적"]
    Yaml["B2 상태 YAML"] --> CLI["tamols_solve"]
    CLI --> Solve
    Codegen["CasADi 코드 생성<br/>개발 시 실행"] --> Kernel["generated/tamols_kernels.c"]
    Kernel --> SQP["SQP 반복 · OSQP<br/>몸통 궤적과 착지점 최적화"]
    Solve --> SQP
    SQP -->|ROS 호출 성공| Sample["몸통·발 궤적 샘플링"]
    SQP -->|CLI 호출| Result["상태·계획 YAML"]
    Sample --> Plan["/terrain_planner/plan"]
    Sample --> Markers["/terrain_planner/markers"]
    Grid --> RViz["RViz"]
    Markers --> RViz
    Plan -.-> WBC["WBC 실행 노드 · 실기/시뮬 공통"]
```

맵 launch는 입력 어댑터·맵·선택적 RViz를 실행한다. `terrain_planner_node`는 별도로
실행해야 한다. 점선은 외부 연동 경계이며, 이 저장소만으로 WBC 추종 완료를 보장하지 않는다.

계획기는 trot의 지지 구간을 구성하고, 이전 계획이 호환되면 warm start를 사용한다.
생성 C 커널로 비용·GIAC·마찰·기구학 제약과 미분을 평가하며, C++에서 QP 구성,
등식 처리, 비선형 제약 보정과 line search를 수행한다. OSQP는 각 QP를 푼다.
SQP가 실행 가능하다고 판정한 계획을 별도 지도·IK 사후 검사 없이 출력한다.

## 저장소 구조

```text
b2_terrain_planning/
├── b2_terrain_planner/
│   ├── src/, include/    # SQP, 지형, RBDL 모델, CLI, ROS 노드
│   ├── codegen/         # CasADi 수식과 C 미분 커널 생성기
│   ├── generated/       # 일반 빌드에서 사용하는 생성 C 코드
│   ├── model/           # B2 URDF와 출처
│   ├── msg/             # LocomotionPlan 메시지
│   ├── config/          # 센서·맵·RViz·B2 상태 설정
│   └── scripts/, launch/# 실기·시뮬 맵 실행
├── scripts/build.sh     # CPU 또는 ROS overlay 빌드
├── docs/                # 맵 운영 절차와 외부 소스 기준
└── patches/             # 외부 CuPy 포크 재현용 패치
```

기존 누적 실험 기록과 Python 참조 구현은 제거했다. 현재 계약 시험과 평지 재현 도구는
`b2_terrain_planner/tests/`에 유지하고, 변경 이력은 Git에서 관리한다.
실행 로그·임시 산출물은 `build/`, `.perception/` 등 Git 제외 경로에 둔다.

## 빌드

CPU 계획기에는 CMake, C++17, Eigen3, yaml-cpp, LAPACK(LP64), pkg-config, tinyxml2,
RBDL 및 URDF reader, **OSQP 0.6.x double-precision C API(PROFILING 포함)**가 필요하다.
기존 환경의 기준은 OSQP 0.6.3, RBDL 3.3.1, tinyxml2 9.0.0이다.
OSQP 1.x는 C API가 달라 직접 호환되지 않는다.

```bash
# 저장소 루트에서 CPU 계획기만 빌드
./scripts/build.sh --cpu-only

# ROS 2 Humble + 맵 overlay 빌드
./scripts/build.sh
```

CPU 산출물은 `build/`, ROS 산출물은 `.perception/`에 생성한다.
`b2_deploy_ws`의 기본 MuJoCo 런처는 워크스페이스 `~/ros2_ws/install`의 노드를 실행하므로
이 런처로 시험할 때는 아래 경로도 빌드해야 한다. `.perception`만 갱신하면 반영되지 않는다.

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --base-paths src --packages-select b2_terrain_planner --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DPython3_EXECUTABLE=/usr/bin/python3
ctest --test-dir build/b2_terrain_planner --output-on-failure
```

CPU 빌드와 실행에는 ROS·GPU·CasADi·IPOPT가 필요하지 않다.
OSQP 위치는 CMake의 `OSQP_INCLUDE_DIR`, `OSQP_LIBRARY`,
RBDL 위치는 `RBDL_DIR` 또는 `CUSTOM_RBDL_PATH`로 지정할 수 있다.

ROS 빌드는 같은 워크스페이스의 `elevation_mapping_cupy`와 `ros2_numpy`도 사용한다.
맵 실행에는 NVIDIA GPU·CUDA·CuPy와 해당 Python 환경이 필요하다.
`B2_WS_ROOT`로 워크스페이스, `ELEVATION_BUILD_ROOT`로 overlay 산출물 위치를 바꿀 수 있다.

## B2 YAML 계획 실행

```bash
./build/tamols_solve --input b2_terrain_planner/config/planner_b2_example.yaml
./build/tamols_solve --speed 0.1 --fast-replanning
./build/tamols_solve --help
```

`model_state`는 base 위치·quaternion, 월드 선속도, 몸통 각속도, 관절 위치·속도를 받는다.
RBDL로 FK·CoM·질량·CoM 기준 관성을 계산한다. 관절 순서는 FL/FR/RL/RR × hip/thigh/calf다.
`model_path`는 입력 YAML 기준 상대 경로 또는 절대 경로이며, 생략하면 포함된 B2 URDF를 쓴다.
`command`는 몸통 기준 `[vx, vy, yaw_rate]`다. `gait_time`, `nominal`, `fixed`도 입력할 수 있다.

정적 지형은 `terrain`의 `frame`, `resolution`, `origin`, `heights`, `variance`, `valid`로
전달한다. 배열은 행=Y, 열=X 증가 방향이며, `origin`은 최소 모서리 좌표다.
입력 `frame`과 지형 `frame`이 일치해야 한다. `terrain_settings`에서 평활 폭·분산 상한·
접촉 오차·지형 비용을 조정할 수 있다. 필드와 기본값은
[terrain.h](b2_terrain_planner/include/b2_terrain_planner/terrain.h)를 따른다.

출력은 상태·시간·제약 위반·모델 정보이며, 성공할 때만 착지점과 구간별 다항식 계수가
포함된다. 좌표 순서는 CoM XYZ, yaw/pitch/roll(ZYX)이고 계수의 시간 단위는 실제 초다.
`converged`, `feasible_stalled`, `feasible_realtime_step`은 각각 수치 수렴, 가능한 해에서의
정체, warm start 이후 제한 반복 갱신을 구분한다. `--time-limit`은 계산 예산이다.
`--profile`은 시간·QP 상태를 추가 출력한다.

`--warm-start <이전 출력.yaml>`은 현재 B2 상태와 함께 이전 계획을 넘긴다.
모델·프레임·지형 사용 여부·보행 주기의 호환성을 확인한 뒤 사용한다. 지도 영역 이동이나
관측 갱신만으로 초기 궤적을 폐기하지 않으며, 이전 착지점은 현재 지도에서 다시 검사한다.
범위·높이·도달성에 맞지 않는 초기 목표는 새 후보를 사용하며, 기존 SQP 제약으로 계산한다.
`--fast-replanning`은 QP 재사용·구조 등식·LAPACK 등을 묶은 CLI 프리셋이다.
ROS 노드의 QP 정규화·반복 수·허용 오차와는 값이 다르다.

합성 모델, 제약 해제, 상세 line-search 추적, 이상 추종 재생 CLI는 제공하지 않는다.
이전 출력의 제약 해제 표시가 참이면 warm start 입력을 거부한다.

## 맵과 ROS 계획 실행

```bash
# 이미 실행 중인 MuJoCo 센서에 맵과 RViz 연결
./b2_terrain_planner/scripts/run_elevation_sim.sh --rviz

# 실기 센서에 맵과 RViz 연결: NIC 이름은 실제 환경에 맞춘다
./b2_terrain_planner/scripts/run_elevation_real.sh --interface enp3s0 --rviz
```

시각 동기화·DDS·장착 TF·입력 만료 처리는
[맵 운영 절차](docs/elevation_map_runtime.md)를 따른다.

계획 노드는 별도 터미널에서 실행한다. 시뮬레이션과 실기 모두 WBC 실행 노드의
`/robot/locomotion_state`를 구독한다. WBC task를 실행해 추정 상태가 유효해져야 계획을 시작한다.
맵 입력 odometry도 `/kf/odom`으로 설정해야 한다. 기본 맵 시연 설정의 `/odom` 또는
`/dog_odom`은 KF 좌표와 자동으로 같아지지 않는다.
아래 예시는 맵의 `elevation_odom`이 상태의 `odom`과 같은 원점·방향으로 설정된 경우다.
노드 내부에서 TF 변환하지 않으며 `frame`은 맵 프레임, `state_frame`은 상태 프레임이다.

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/local_setup.bash
ros2 run b2_terrain_planner terrain_planner_node --ros-args \
  -p state_topic:=/robot/locomotion_state -p state_frame:=odom -p frame:=elevation_odom
```

발행 계획·마커는 상태 메시지의 `header.frame_id`를 사용한다.
RViz의 Fixed Frame과 연결되는 TF도 같은 좌표 계약에 맞춰야 한다.

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

## 현재 ROS 연결의 구현 범위

- `LocomotionState`의 전신 CoM 위치·속도와 네 발 위치를 최적화 초기 상태로 사용한다.
  WBC 실행 노드는 실기에서도 얻는 IMU·관절 q/dq·모터 토크 추정값과 KF/운동학만 사용한다.
  simulator GT base·발·접촉·GRF는 이 메시지의 입력이 아니다.
- 몸통 자세와 world 각속도를 받아 Euler 변화율로 변환한다. 명목 base→CoM 변환은 제거했다.
  질량·관성·도달성 모델의 명목 관절 근사는 여전히 남아 있다.
- 네 발 속도와 `contact_estimated`, `contact_confirmed`를 전달한다. WBC가 착지 확인에 따라
  보행 시간을 정지·재개하며 TAMOLS는 같은 `gait_time`으로 고정 trot 순서를 계획한다.
  보행 대기 지지는 WBC stance를 나타내는 `contact_confirmed`만으로 판정한다.
  stance 이후 `contact_estimated`가 OFF여도 지지를 취소하지 않으며, 추정 접촉은
  접촉 주변 지도 보정에 별도로 사용한다. 현재 지지발의 높이를 지도와 강제로 일치시키지 않는다.
  현재 지지발은 측정 XYZ에 지지가 있다고 가정하며, 스윙발도 현재 XY 아래 지도 관측을
  요구하지 않는다. 현재 발 아래 미관측으로 SQP 시작을 거부하지 않는다.
  몸통 최적화는 각 구간의 실제 지지발 조합과 위치에 맞춰 대각선 2발·3발·4발
  제약을 선택한다. 착지 시차로 생기는 3발 지지는 네 조합의 삼각형으로 계산하며,
  미착지 발을 대각선 지지로 대신 사용하지 않는다. 3발·4발 구간에는 지지 여유
  변수를 사용한다. 이륙 순서는 trot을 유지하고 착지점은 해당 발의 이륙 시 확정한다.
- 메시지 필드명은 유지한다. `base_pose.position`과 `base_twist.linear`는 전신 CoM,
  `base_pose.orientation`은 몸통 자세, `base_twist.angular`는 roll/pitch/yaw 변화율이다.
  `b2_deploy` bridge가 각속도로 변환하고 WBC는 전신 CoM Jacobian으로 병진을 직접 추종한다.
  base 원점 궤적을 이 메시지의 병진 필드에 넣지 않는다.
- 스윙 발은 위치·속도 경계를 갖는 Hermite 곡선으로 생성한다. 이륙 전에는 재계획 입력의
  측정 발 위치로 시작점을 갱신한다. 이전 후보와 착지 시각이 같아도 이전 시작점·정점을
  복사하지 않으며, 정점 높이는 최신 시작점과 착지점에서 다시 계산한다.
  WBC 보행시각이 현재 스윙에 진입하면 실행 계획의 곡선을 고정한다. 진행 중인 발은 착지점 XYZ를
  최적화 등식 제약으로 고정하고 남은 곡선도 유지한다. 새 명령은 아직 이륙하지 않은 발의
  착지점에 반영한다. CoM은 고정된 착지점을 제약에 사용하면서 계속 재계획한다.
  확정 착지점은 지도 관측이 사라져도 전달받은 XYZ를 유지한다. 지도 조회 거부와
  높이·경계 비용 재조회를 적용하지 않으며 XYZ 고정 등식은 유지한다. 아직 확정하지
  않은 새 착지점은 기존 관측 지형 검색과 지형 비용으로 계산한다.
  전체 관절 궤적·스윙 충돌 회피를 계산하는 구현은 아니다.
- 상태와 지도의 시각 만료를 solve 전후 검사한다. 미래·비정상·다른 프레임의 상태와
  중복·역순 시각을 거부한다. 같은 상태로 계획을 반복 발행하지 않고 원래 상태 시각을 보존한다.
  `require_map=false`는 맵 없는 평지 시험용이다. 이미 수신한 맵이 만료되면 이 모드도 계획을 중단한다.
- TF 변환은 수행하지 않는다. `frame`과 `state_frame`의 이름이 달라도 수치 원점·축은 같아야 한다.
  gait 시간은 유효한 WBC 보행 시계를 우선 사용한다. 새 WBC 세션으로 KF 원점이 초기화되면
  계획기와 지도도 재시작해 기존 좌표·계획을 비워야 한다.
- GIAC는 계획 시작 자세의 관성을 고정하는 SRBD 근사다. SQP 도달 길이 제약은
  실제 관절별 IK 검사와 같지 않으며, 이번 변경으로 별도 최종 IK 검사는 수행하지 않는다.

위 항목은 현재 구현의 연결 경계다. CoM 추종 연결과 실제 지형 폐루프 검증은 구분한다.

## 확인 범위

정리 후 CPU Release 빌드와 ROS 메시지·노드의 CMake 빌드·설치를 확인했다.
정지·보행·고속 프리셋·평탄 높이맵·잘못된 입력·warm start의 6개 입력을 정리 전과
비교해 종료 상태와 반복 수가 같음을 확인했다. 각 구간 101개 표본에서 CoM 위치·자세의
최대 절댓값 차이는 약 `2.3e-9`(각 성분의 m 또는 rad), 착지점 차이는 `1.9e-10 m` 이하였다.
새 출력 형식의 warm start, QP 상태 출력, 폐기 옵션과 과거 제약 해제 계획의 거부도 확인했다.

ROS 구성 후 `colcon`이 다음 단계로 진행하지 않아 CMake 직접 빌드·설치로 확인했다.
설치된 CLI 실행과 ROS 노드의 공유 라이브러리 연결을 확인했으며, 이번 정리에서
GPU 맵·RViz·실기 WBC 통합 실행은 수행하지 않았다. 상세 변경·검증 내역은 Git 커밋에 남긴다.

2026-10-16 CoM 입력 보정 후 `terrain_planner_node` 빌드를 확인했다.
격리 ROS domain에서 합성 base odometry z=0.5120595 m를 보내고, `require_map=false`,
정지 명령으로 발행한 계획의 첫 CoM 위치가 명목 모델 계산값
`[-0.00124548, 0.00252666, 0.49859984]` m와 일치함을 확인했다.
이는 상태 변환·메시지 발행 검증이며 GPU 맵·MuJoCo 폐루프 보행 검증은 아니다.

## 커널 생성과 외부 소스 관리

`codegen/generate_kernels.py`는 Python 참조 구현 없이 독립적으로 C 커널을 생성한다.
일반 빌드에는 이미 포함된 `generated/`를 사용한다. 수식 변경 시 CasADi 3.7.2와 NumPy가
있는 환경에서 다음 명령을 실행하고 생성 C·헤더·manifest를 함께 검토한다.

```bash
python3 b2_terrain_planner/codegen/generate_kernels.py
```

B2 모델의 기준 커밋·해시는 [모델 출처](b2_terrain_planner/model/README.md),
외부 맵 저장소의 기준은 [source_manifest.json](docs/source_manifest.json)에 기록한다.
`patches/elevation_mapping_cupy.patch`는 외부 CuPy 포크의 발밑 보호·정책 크롭을
재현하는 자료다. 기존 환경에는 적용되어 있으므로 중복 적용하지 않는다.
새 환경에서는 기록된 기준 커밋에서 `git apply --check` 후 적용한다.
다른 로봇 저장소는 별도 요청 없이 수정하거나 동기화하지 않는다.

2026-10-16 공통 상태 토픽 연결 검증: `LocomotionState` 생성과 계획기 빌드,
실제 ROS 노드 통합 시험(`b2_terrain_planner/tests/locomotion_state_ros_test.py`)을 통과했다.
전달한 CoM·발 위치가 첫 계획에 반영되는지, 원래 시각 보존, NaN·잘못된 자세/프레임·미래/오래된
상태·중복/역순 메시지 거부, 상태/지도 만료 시 발행 중단을 확인했다.
통합 시험은 메시지 계약 검증을 위해 time_limit=3초, 입력 유효기간=4초로 실행했다.
초기 0.5초 제한에서는 지형 solve가 time_limit으로 실패했으므로 기본 10 Hz 성능이나
MuJoCo 지형 폐루프 보행 성공을 의미하지 않는다. 온보드 배포·실기 구동은 수행하지 않았다.

```bash
# 별도 ROS domain에서 실행: 실제 WBC에 연결하지 않는 합성 상태/지도 시험
ROS_DOMAIN_ID=174 RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  /usr/bin/python3 src/b2_terrain_planning/b2_terrain_planner/tests/locomotion_state_ros_test.py \
  install/b2_terrain_planner/lib/b2_terrain_planner/terrain_planner_node
```

외부 계획 추종은 WBC 착지 확인에 따라 보행 시간을 정지·재개한다. `LocomotionState/Plan`의 보행 시간 필드를 추가했으므로 두 패키지 재빌드 후 양쪽 실행기를 재시작해야 한다. 평지 폐루프 안정성 검증 결과는 각 패키지 운영 문서를 참조한다.

TAMOLS 추종의 다음 스윙은 이륙 후에도 확인 지지발이 최소 두 개 남을 때 허용한다. 부족하면 이륙 직전에서 기다리며, 대각쌍 대기 시간 0.3초로 종료하지 않는다. 발별 탐색 제한과 계획 만료는 별개다.

## 계획 갱신 중단 진단

기본 실행에서는 진단 토픽뿐 아니라 `~/.ros/log/`(`ROS_LOG_DIR` 지정 시 해당 경로)의
planner 로그에도 `TAMOLS_PIPELINE`/`TAMOLS_SOLVE` 뒤 JSON을 자동 저장한다.
INFO 로그와 ROS 외부 파일 sink가 활성화되어 있어야 한다. 계산 시작·결과·발행 거부는
throttle 없이 기록하며, 동일 계산/부모의 반복 보류만 최초 및 1초마다 저장한다.
따라서 파일의 pipeline `event_seq` 간격은 반복 보류 생략 때문일 수 있다. 토픽은 그대로 발행한다.

`TAMOLS_SOLVE`의 `optimization_started`는 SQP 반복 실제 진입 여부다.
`stage`는 반환 결과의 실패 단계이며 `input_validation`(입력·설정),
`problem_setup`(지도 조회·문제 구성), `initialization`(초기값),
`optimization`(SQP), `final_validation`, `terrain_validation`, `kinematics_validation`,
`export`, `total_time_budget`을 구분한다. 성공은 `complete`다. worker 밖 예외는
`stage=not_started`, `status=solve_exception: ...`로 기록될 수 있으며 진입 여부를
확정할 수 없는 예외로 취급하며 `optimization_started=null`을 기록한다. `terrain_attempts`에는 내부 지지면 재시도 결과도 남긴다.

`command`, `touchdown_stop`, `contact_confirmed`, `contact_estimated`,
`parent_plan_stamp_ns`, `prefix_duration`은 해당 계산 입력 snapshot이다.
`published`/`publication_reason`으로 계산 성공과 발행 성공을 구분한다.
`attempt_seq`로 시작/결과를 연결하고 입력 `state_stamp_ns`를 B2 ROS 로그의
`TAMOLS_WBC` 계획 stamp 및 WBC CSV `plan_stamp_ns`와 연결한다. ns는 정수로 읽는다.
B2의 `adoption_observed`는 ROS 주기로 관측한 제어 판정 변화이며 모든 제어 틱 기록은 아니다.
이 기록은 과거 2초 제한 경고만으로 마지막 실패 단계를 확정할 수 없던 문제를 보완한다.

`/terrain_planner_node/pipeline_status`는 JSON 문자열로 각 보류·solve·발행 이벤트를 기록한다.
`event_seq`는 이벤트 순번, `attempt_seq`는 Solve 호출 순번이다. `ros_ns`, `state_stamp_ns`,
`map_stamp_ns`, `gait_epoch_ns`, `gait_time`, `solve_start_ns`, `solve_end_ns`,
`last_publish_ns`, `last_published_stamp_ns`로 입력부터 발행까지 연결한다.
solve 시각은 최근 시도에 속하며 `solve_started`의 종료 시각 0은 아직 종료되지 않았음을 뜻한다.

- `deferred`: `no_state`, `state_not_fresh`, `map_not_fresh`, `no_map`,
  `state_already_planned`, `support_unconfirmed`(발 번호 포함), `euler_singularity`,
  `active_swing_reference_missing`, `active_plan_not_in_history`, `active_body_horizon_too_short`.
- `solve_started` / `solve_failed`: 계산 시작 또는 solver의 실패 status. `time_limit`은 해 부재의 증거가 아니다.
- `publish_deferred`: solve 성공 후 `state_not_fresh_after_solve`,
  `map_not_fresh_after_solve`, `state_reset_during_solve`,
  `active_plan_changed_after_solve`, `body_handover_missed`, `contact_phase_changed_after_solve` 중 처음 탈락한 조건.
- `published`: 실제 plan publish 호출 반환 직후. 수신부 채택과는 별도 단계다.
- `state_rejected`: 상태 입력 검증 실패 또는 중복/역순. `received_stamp_ns`는 거부된 입력 시각이다.
- `reset`: ROS 시각 역행.

기존 `solve_status`는 solve 결과만 발행하며 `attempt_seq`, 입력 stamp와 solve 시작·종료 시각을
추가한다. `published=true`는 실제 publish 호출 반환 이후 발행한다.
plan header stamp는 여전히 입력 상태 시각이며 watchdog·freshness 조건은 유지한다. 새 명령 수신으로 계산 중인 snapshot을 취소하지 않는다.
Solve는 입력·설정·직전 성공해의 복사본으로 별도 작업에서 한 번에 하나만 실행한다.
상태·지도·명령 수신과 결과 채택·진단 발행은 단일 planner executor에서 직렬 처리한다.
완료는 2 ms 타이머로 확인하며, 실패·발행 보류 시 최신 입력으로 즉시 재시도한다.
동일 상태 stamp의 즉시 재시도는 한 번으로 제한하고, 이후 새 상태 수신 때 재개한다.
미확인 예정 지지발·입력 freshness 조건은 재시도에도 동일하게 적용한다.
`body_clock_valid` 상태에서는 미확인 착지발을 지지로 가정하지 않고 현재 확인된
두 개 이상 지지발을 유지하는 0.8초 몸통 계획을 계산한다. `hold_gait` 동안 발 위상만
정지하고 몸통 시간은 계속 진행한다. legacy 단일 시계 입력은 미확인 지지를 보류한다.
계산 중 epoch·보행 시각 초기화·접촉 위상·실행 계획이 바뀌면 결과를 발행하지 않는다.
WBC stance와 보행 대기 상태가 유지되면 추정 접촉 변동만으로 대기·재개 계획을 버리지 않는다.

착지 예정 발이 아직 WBC stance로 확인되지 않으면 즉시 TAMOLS의 수평 이동·yaw 속도
목표를 0으로 낮춘다. 실제 궤적은 기존 SQP가 계산하며 고정 앞부분과 p/v 인계는 유지한다.
착지가 확인되면 대기 중에도 계속 갱신한 사용자 램프 값을 사용한다. 추정 접촉 ON만으로 대기를 해제하지 않는다.
기존 길이 여유 감속과 발행 정체 감속은 사용하지 않는다.

- 정지 시각의 속도·가속도 등식, 정지 후 위치 고정, 추가 2cm 여유, 정지 전용 구간 분할·
  초기값, 12등분 평가와 48등분 사후 검사를 제거했다. 기본 2회·6등분을 사용한다.
- 착지 대기에도 호환되는 이전 계획을 재사용한다. 기존 해를 재사용하면 2회 이후
  실행 가능한 개선 결과에서 조기 종료한다.
- 최적화 뒤 지도 높이와 관절 IK 검사로 다시 거부하지 않는다. 호환용 출력의
  `terrain_validation`, `kinematics_valid`는 미측정이며 `ik_checks`는 0이다.
- 실패·종료 후 전체 제약을 재평가하지 않는다. 진단에는 마지막으로 계산한 평가값을 쓴다.
  SQP/QP 내부 100ms 시간 제한은 유지하고, 출력·진단 후 경과시간을 이유로 성공을 취소하지 않는다.
- 목표 속도 0은 정지 시각이나 실제 추종을 강제하는 조건이 아니다. 별도 만료 복구는
  이 변경에 포함하지 않는다.

`pipeline_status`와 `TAMOLS_SOLVE`의 `touchdown_stop`은 목표 속도 0 적용 여부다.
`touchdown_stop_entered/released`, `effective_command`로 진입·해제를 확인한다.
정지 시한·추가 여유 파라미터와 로그 필드는 제거했다. 계산 중 대기 진입·해제가 바뀐
후보의 `touchdown_stop_changed_after_solve` 발행 보류는 기존대로 유지한다.

실패·발행 보류는 `previous_`를 지우지 않는다. 마지막 발행 성공해를 다음 초기값으로 넘기고,
Solve 내부의 보행 시각·모델·지도 호환성 검사와 착지점 재검사를 그대로 거친다.
epoch/보행 시각 초기화 또는 ROS 시각 역행 시에는 성공해와 스윙 참조를 초기화한다.
이 보존은 WBC 계획의 stamp/horizon을 연장하거나 만료된 계획을 재발행하는 기능이 아니다.
`last_planned_ns`는 최근 **시도한** 상태 stamp이고, `solve_state_stamp_ns`는 해당 solve 입력이다.
비동기 계산 중 `state_stamp_ns`는 더 최신 수신 상태일 수 있으며 `retry_requested`로 재시도 대기를 구분한다.
동일 실행에서 pipeline/solve 상태, plan, `/robot/locomotion_state`, `/robot/locomotion_command`,
`/terrain_planner_node/effective_command`, 사용 map, `/clock`, `/rosout`과 WBC 수신 진단·CSV를 기록한다.

검증: 전체 빌드와 6개 회귀 테스트, 별도 ROS domain의 `locomotion_state_ros_test.py`를 통과했다.
통합 시험은 state/map 만료, gait pause, 미확인 지지발 보류 및 publish 시각 순서를 검증한다.
별도 MuJoCo rosbag에서 solve 175회와 발행 173회가 추적됐으며 pipeline 이벤트 순번 누락은 없었다.
해당 실행의 첫 damping은 유효 계획 상태의 WBC QP 실패로, 2026-10-17 사용자 로그와 구별한다.

2026-10-18 실패 복구 수정 검증: 전체 빌드와 7개 CTest, ROS 상태 계약 시험,
진행 중 스윙 궤적 고정 시험을 통과했다. `planner_retry_test`는 계산 완료를 지연시키고
실패를 주입해 수신 지속·성공해 보존·최신 상태 즉시 재시도·동일 입력 재시도 제한,
epoch/pause/명령 변경 및 시각 역행 중 이전 결과 발행 차단을 검증한다.

별도 domain 187의 MuJoCo 평지 시험은 전진 명령 20초를 포함한 약 24.024초의 WBC CSV
8,729행에서 damping 없이 끝났다. rosbag 기준 solve 190회, 발행 156회이며,
`time_limit` 3회 뒤 모두 이전 성공해를 재사용해 실패 이벤트 후 36.04/32.26/47.99 ms에
새 계획을 발행했다. 초기 발 높이 검사 실패 2회도 후속 재시도에서 회복했다.
계산 중 접촉 위상 변경 28회와 명령 변경 1회는 발행을 보류했다. 평균 solve 시간은
41.94 ms로, 계산 자체의 성능 개선을 검증한 시험은 아니다. 진단 이벤트 1,282개의
순번 누락은 없었다. 산출물은 워크스페이스 `artifacts/planner_retry_20261018/`에 있으며,
이 시험은 사용자 실행 `111947.csv`와 별개이고 모든 보행 조건의 안정성을 보장하지 않는다.


## 명령 확정과 실행 계획 연결

입력 명령은 최신 요청으로 보관하고, TAMOLS에 넣는 속도 목표는 매 상태 수신 시 램프한다.
네 발 지지나 반걸음 경계를 기다려 명령을 한 번에 확정하지 않는다.
`command_ramp.linear_accel`(기본 0.5 m/s²)은 XY 속도 벡터 변화량의 크기를 제한하고,
`command_ramp.angular_accel`(기본 1.0 rad/s²)은 yaw 속도 변화율을 제한한다.
예를 들어 전진 +0.5에서 후진 −0.5 m/s로 바꾸면 목표는 약 2초에 걸쳐 반전한다.
이 값은 사용자 목표의 변화율이며 실제 몸통 가속도를 강제하는 최적화 제약이 아니다.

램프는 WBC `body_time`으로 진행하고 legacy 입력은 상태 header 시각을 사용한다.
같은 상태의 재시도는 램프를 진행시키지 않으며 epoch·시각 초기화 시 목표를 0부터 시작한다.
명령 유효기간이 지나면 램프 목표를 0으로 바꾼다. 착지 대기는 기존대로 즉시 목표 0을
우선한다. 이때 사용자 램프를 초기화하거나 멈추지 않고 확인 후 그 값을 다시 사용한다.
따라서 반복 착지 대기 때문에 사용자 램프가 매번 0부터 시작하지 않는다. 비상정지를 대체하지 않는다.
solver는 램프 목표의 복사본으로 끝까지 계산하고, 계산 중 새 명령이나 램프 진행을 이유로
성공 결과를 폐기하지 않는다. 최적화 제약·반복·발행 조건은 이 램프로 추가하지 않는다.
`~/effective_command`와 `TAMOLS_SOLVE.command`에서 실제 입력을 확인한다.
0.8초는 계획 horizon이며 명령 갱신 타이머가 아니다.

`LocomotionState.active_plan_stamp_ns`는 WBC가 실제 사용한 계획이다. 최근 발행된 후보와
구분하며, 이 확인에 맞는 몸통·스윙 참조를 사용한다. 후보에는 부모 stamp와
`handover_body_time`이 포함된다(legacy는 `handover_gait_time`). WBC 제어 주기는 부모·epoch·시각·연속성을 검사해
후보를 채택하고, 부적합 후보가 와도 기존 계획을 유지한다. 기존 계획의 실제 ROS
만료는 그대로 적용한다. header stamp는 계산 입력 상태의 시각이다.

이동 중에는 기본 설정의 몸통 연결점이 입력 body time + time_limit + 2*sample_dt, 즉 140 ms 뒤다.
그 이전에는 부모 몸통 다항식을 정확히 절단해 사용한다. 이후 궤적은 연결점의 위치·속도
등식 제약과 가속도 차이 비용을 포함해 최적화한다. 가속도 연속성을 보장하는 계약은 아니다.
연결점을 위상 내부에 추가하더라도 접촉 일정과 현재 스윙 종료 시각은 바꾸지 않는다.
WBC가 사용하는 다항식은 `body_breaks/body_coefficients`로 전달되어 p/v/a 및 물리 각속도를
같은 원함수에서 계산한다. 발 가속도 feedforward는 추가하지 않는다.

`LocomotionState/Plan`의 `body_clock_valid/body_time`은 발 위상과 별도인 몸통 시각이다.
착지 대기에도 진행하며 몸통 연결점은 현재 실행 곡선의 미래 p/v로 정한다.
`gait_hold_duration`만큼 현재 지지를 유지한 뒤 발 위상을 재개하는 접촉 일정을 계산하고,
전체 horizon은 0.8초로 유지한다. 미확인 착지가 있으면 전체 horizon 동안 현재 지지를 유지한다.
확인 뒤에는 `gait_resume_body_time`에 재개한다. 채택한 재개 계획을 다시 갱신할 때
이 시각을 뒤로 밀지 않으며, 발 배열의 유효 구간은 `gait_duration`으로 제한한다.
기존 곡선을 horizon 밖으로 계속 연장하지 않는다. 두 패키지를 함께 재빌드하고 재시작해야 한다.

몸통 앞부분의 계수는 기존 실행 곡선으로 고정하고, 새 SQP의 동역학 제약은 연결 이후
구간에 적용한다. 실행하지 않을 보조 곡선을 별도로 계산하지 않는다. 측정한 CoM·자세는
명목 발 배치와 명령 좌표에, 측정한 발·접촉·지도는 지지 조건과 새 착지점 계산에 사용한다.
몸통 기준점은 연속성을 위해 기존 기준 곡선에서 가져오며 측정 CoM으로 순간 교체하지
않는다. 실제 CoM 추종오차는 WBC의 피드백으로 보정한다. 이어 붙인 몸통에 별도의
최종 IK 검사를 수행하지 않는다.

WBC는 새 이륙 후 확인 지지발 두 개 이상 조건에 더해, 마지막 착지 확인 이후 상태가
반영된 실행 계획과 새 몸통 구간의 적용 가능 여부를 요구한다. 새 계획을 받았어도
이전 몸통을 유지하는 앞부분에 있으면 이륙을 기다린다. 계획 대기 중에도 ROS watchdog은 흐른다. 따라서 긴
접촉 대기, 반복 solve 실패, 수신 지연은 여전히 명시적 만료로 종료될 수 있다.
착지 대기에도 WBC는 몸통 p/v/a를 계속 추종한다. 채택 시 부모 곡선의 p/v/a 일치를 검사하며, 새 최적화 구간의 연결은 p/v 연속성을 보장한다.

상태 메시지의 `candidate_plan_stamp_ns/plan_adoption_status`는 수신과 실행 채택을
구분한다. 상태 값은 0 없음, 1 채택, 2 부모 불일치, 3 연결 시각 경과, 4 무효 샘플,
5 epoch 불일치, 6 몸통 위치·속도 연속성 불일치다. CSV의 `plan_accepted_*`는
ROS callback의 후보 수신 승인이고 실제 실행 stamp는 `plan_stamp_ns`다.

연결 이후 새 몸통 구간 사이에는 기존 위치·속도 연속 조건을 유지한다. 부모 궤적과의
연결점은 위치·속도 등식과 가속도 차이 비용을 사용한다. 초기값은 실제 이어 붙인 결과다. 고정 앞부분은 기존 계수를 정확히 유지하며,
정지 인계에는 이전 다항식의 구간 경계를 보존한다.

시간 제한으로 중단된 유한한 SQP 반복값은 `seed_available`로 표시해 별도 재시도
초기값으로 보관한다. 마지막 성공해와 실행 계획을 덮어쓰지 않으며, 같은 부모 계획과
세션에서만 재사용한다. 재시도도 기존 SQP 제약의 실행 가능 판정을 통과해야 발행한다.
`solve_status.warm_start`의 `shifted_retry_iterate`는 이 경로이며 계산 중간값의 실행을
뜻하지 않는다. 정지 인계에서는 부모 다항식의 구간 경계도 유지해 초기값을 옮긴다.

보행 시간은 미확인 착지발이 생긴 명목 착지 경계에서 멈춘다. 실제 착지와 해당 상태를
반영한 몸통 인계가 끝난 뒤 네 발 지지 구간을 진행하므로, 착지 탐색이 지지 시간을
미리 소모하지 않는다. 한 번 반영한 착지 피드백은 이후 후보의 고정 앞부분에도 계승되며,
매 후보의 미래 연결점 때문에 같은 착지를 다시 기다리지 않는다.

2026-10-19 평지 MuJoCo 검증: 전진→후진→전진 시험 두 회의 CSV 26.05초·36.14초
전체에서 damping latch가 없었다. 마지막 시험은 전진 7.5초, 1초 전환, 후진 15초,
1초 전환, 전진 7.5초이며 전후 정지 명령을 포함한다. 실제 planner 입력의 후진 명령은
-0.30 m/s다. 318회 계산 중 252개 계획을 발행했고, time_limit 10회 뒤에도 모두
다음 발행으로 복구했다. 최대 계획 ROS age는 0.629초로 기존 0.8초 watchdog 안이었다.
계산 평균/95%/최대는 47.6/86.0/104.1 ms이며, 시간 초과 결과는 발행하지 않았다.
진단 이벤트는 1~2107번 사이 누락이 없고 실행 채택 거부도 관측되지 않았다.

동일 시험에서 WBC 추정 좌표계의 수평 CoM 오차는 중앙값 7.83 mm, 95% 19.12 mm,
최대 28.07 mm였다(활성 계획·CoM task, CSV t>1.5초). 계획이 바뀐 인접 제어 tick의
몸통 목표 이동은 최대 0.652 mm이며 정상 이동량도 포함하므로 순수한 인계 불연속
크기와 같지는 않다. 명령/조건이 다른 과거 로그와 개선율로 비교하지 않는다.

계획기 7개·WBC/bridge 4개 회귀 시험, 상태 입력 계약 ROS 시험, 스윙 고정·명령 확정·
착지 인계 ROS 시험을 통과했다. 기록은 workspace의
`artifacts/handover_20261019/verification.json` 및 `reversal_final/`에 있다.
검증 범위는 평지 MuJoCo이며 실제 로봇과 비평지 검증은 별도다.
