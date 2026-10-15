# Agent Workflow Defaults

## b2_deploy_ws 저장소 적용 범위
- 이 저장소는 pRbLibrary 소스를 일반 파일로 포함한다. 아래 공통 규칙 중 서브모듈 포인터 갱신 절차는 이 저장소에 적용하지 않는다.
- pRbLibrary 기준 커밋과 갱신 방법은 루트 README의 운영 원칙에 기록한다.
- 기존 로그 정리 방침에 따라 개발 변경 이력은 Git 커밋에 기록하고, 과거 로그 파일은 다시 추가하지 않는다.
- 다른 로봇 저장소와의 동기화는 해당 저장소 수정이 별도로 요청된 경우에만 수행한다.

## Edit Approval Policy
- 코드 수정 전 반드시 변경 계획을 먼저 보고한다.
- 보고 내용에는 변경 목적, 대상 파일, 예상 diff 요약을 포함한다.
- 사용자 승인 이후에만 실제 파일을 수정한다.
- 변경 계획 보고는 승인으로 간주하지 않는다. 사용자의 명시적 승인 표현 이후에만 수정한다.
- 명시 승인 전에는 `apply_patch`, 포맷터, 코드 생성, 파일 생성/삭제/이동, 대량 치환 등 실제 파일 내용이 바뀌는 모든 행위를 금지한다.
- 승인 범위는 보고한 대상 파일과 예상 diff 범위로 제한한다. 범위가 바뀌거나 추가 수정이 필요하면 다시 계획을 보고하고 재승인을 받는다.
- GUI/앱 화면 수정은 시각 구조와 사용 흐름이 바뀌기 쉬우므로, 작은 polish 작업이라도 수정 전에 계획과 대상 파일을 보고하고 승인받는다.
- 수정 중 빌드 실패, 문법 오류, 깨진 레이아웃 등 회귀가 발생하면 추가 기능 진행보다 복구 계획 보고와 승인 절차를 우선한다.
- “가능할까?”, “할 수 있을까?”, “어떻게 하면 될까?” 같은 가능성·설계·의견 질문은 수정 승인으로 간주하지 않는다.
- 사용자가 “적용해”, “수정해”, “진행해”, “OK”처럼 실제 변경을 명시적으로 승인한 경우에만 파일을 수정한다.
- 요청이 질문인지 변경 지시인지 애매하면 분석과 수정안만 제시하고 승인 여부를 물은 뒤 기다린다.

## Code Style
- 신규 작성 및 수정 코드는 Google 스타일 가이드를 따른다.
- `README.md`를 포함한 사용자 대상 문서는 기본적으로 한글로 작성한다.
- 각 `*.cpp`/`*.h` 수정 시 파일 최상단 헤더의 `@version`은 패치 단위로 올린다.
- 파일 헤더에는 해당 파일의 역할, 주요 기능/구조를 이해할 수 있는 설명을 유지/보강한다.
- 각 `*.cpp`/`*.h` 파일 헤더의 `@update` 설명 문구는 기본적으로 한글로 작성한다.
- 파일 헤더 `@update` 항목은 최신 2개만 유지한다(오래된 항목은 정리).
- 수정 내역의 상세 로그를 헤더 주석에 누적 기록하지 않는다.
- 실제 수정 내용은 `git commit` 메시지/히스토리에 반드시 반영한다.
- `git commit` 메시지는 한글/영문을 함께 작성하는 것을 원칙으로 한다.
- 파일 헤더 `@version`은 기본적으로 `0.0.x` 패치만 증가시킨다.
- `0.x.0`(minor) 버전 증가는 사용자가 명시적으로 요청한 경우에만 수행한다.
- `0.0.x` 패치 번호는 필요 시 `999`까지 증가 가능하다.
- C++를 포함한 모든 코드와 문서 예시 코드에서 삼항 조건 연산자 `?:` 문법을 사용하지 않는다.
- 조건 분기는 `?:` 대신 명시적인 `if/else`, 별도 함수, lookup table 등 읽기 쉬운 구조로 작성한다.

## 에러 기록 규칙
- 확인한 원인, 해결과 검증 결과는 Git 커밋과 관련 운영 문서에 기록한다.
- 실행 로그와 임시 산출물은 `.gitignore`로 제외한다.

## pRbLibrary 운영 관리
- `pRbLibrary`는 범용 제어기 소스를 `src/pRbLibrary/`에 일반 파일로 포함해 관리한다.
- 이 저장소에는 pRbLibrary 서브모듈을 등록하지 않는다.
- 원본 갱신 시 기준 커밋을 기록하고 폴더 구조와 공통 제어 계약을 유지한다.
- `pRbLibrary` 내부 폴더를 분해해 패키지 상위 `src/`, `include/`, `runtime/`, `threads/`로 따로 빼내지 않는다.
- `pRbLibrary` 수정은 반드시 `pRbLibrary` 저장소에서 커밋/푸시 후, 각 패키지에서 서브모듈 포인터를 업데이트한다.
- 로봇별 런타임 설정, 시뮬레이터 연동 정보, Isaac Sim 관련 메타데이터는 가급적 각 패키지 상위 단의 YAML에서 관리한다.
- 패키지 특화 요구를 해결하기 위해 `pRbLibrary`를 먼저 수정하는 접근은 지양하고, 우선 패키지 상위 단 설정/YAML로 흡수 가능한지 검토한다.
- `pRbLibrary` 수정은 여러 로봇/패키지에 공통으로 필요한 범용 기능, 공통 계약, 공통 버그 수정에 한해 최소 범위로 수행한다.
- 로봇 패키지 작업을 시작하기 전에 해당 패키지의 `pRbLibrary` 서브모듈 상태를 먼저 확인한다.
- 확인 항목은 최소 다음을 포함한다:
  - 현재 서브모듈 포인터 SHA
  - 현재 체크아웃 브랜치와 upstream 추적 상태
  - 로컬 수정 유무
- `pRbLibrary`에 원격 최신 변경이 반영돼야 하는 작업이라면, 먼저 `pRbLibrary` 저장소 최신 상태를 반영한 뒤 그 기준으로 패키지 작업을 시작한다.
- 패키지 작업 중 `pRbLibrary` 변경 필요가 확인되면, 패키지 내부에서 임시 수정하지 말고 `pRbLibrary` 저장소에서 먼저 수정/커밋/푸시한 뒤 서브모듈 포인터를 갱신한다.
- `pRbLibrary` 수정 작업을 시작하기 전에는 반드시 `pRbLibrary/docs/01. RBLIBRARY_GUIDE.md`를 먼저 읽고, 이어서 `pRbLibrary/docs/02. PACKAGE_STRUCTURE.md`를 확인해 상세 운영 원칙과 package glue code 경계를 재확인한다.

## 제어 프로파일 네이밍 규칙
- 목적별 제어 프로파일은 각 로봇 패키지 루트의 `config/profiles/` 하위에 저장한다.
- `pRbLibrary/config/profiles/`는 active profile 저장소로 사용하지 않는다.
- 파일명 규칙은 `{robot}_{purpose}_{mode}_v{major}.yaml`을 따른다.
- `robot`은 로봇 키를 사용한다. 예: `h1_2`, `b2`, `z1`
- `purpose`는 목적 키를 사용한다. 예: `default`, `rl`, `walk`, `task`, `sim`, `real`
- `mode`는 계산 모드를 사용한다. 예: `online`, `offline`
- `v{major}`는 호환성 깨짐(필드 삭제/의미 변경) 시에만 증가시킨다. 예: `v1`, `v2`
- 미세 튜닝(값 변경만) 시 파일명을 유지하고 내부 메타데이터(`profile_version`, `updated_at`)만 갱신한다.
- 패키지 실행 시 어떤 프로파일을 사용할지는 각 패키지의 `config/robot_config.yaml`에서 선택한다.
- GUI 개발자 모드는 해당 package-local profile YAML을 열고 저장/적용할 수 있다.
- GUI 모니터 모드는 task 선택과 실행은 가능하지만, profile YAML 저장/적용, 초기 자세 수정, gain tuning 같은 중요 파라미터 변경은 할 수 없다.

## Task 추가 운용 규칙
- task 추가는 `config/tasks/*.yaml`의 task contract와 `runtime/task`의 C++ task handler registry 등록을 한 쌍으로 수행한다.
- YAML에 추가된 `tasks[].name`은 반드시 `runtime/task`의 C++ registry에 등록돼야 한다.
- 등록되지 않은 task가 YAML에 존재하면 빌드 단계 validator가 실패해야 한다.
- `runtime/task`는 task 계약, 검증, handler 등록 여부를 관리한다.
- 실제 제어 알고리즘은 task 성격에 따라 `pRbLibrary` 공통 제어 계층 또는 로봇별 runtime glue 계층에 둔다.
- GUI/ROS2 bridge는 task 실행 로직을 직접 소유하지 않고, task contract와 runtime input 전달만 담당한다.
- 새 runtime state가 필요한 경우에만 `OnlineState`, `runtime/state` registry, ROS2 mode 계약을 함께 갱신한다.
- 일반 task 추가는 FSM을 수정하지 않는 것을 원칙으로 한다. FSM 수정은 상태 진입/이탈 side effect가 필요한 경우에만 허용한다.
- `config/tasks/*.yaml`의 `runtime_state`, `allowed_from`, `allowed_to`, `completion.next_state`는 `runtime/state` registry에 존재하는 state 이름만 사용한다.
- task YAML만 추가해 GUI 버튼이 보이게 만드는 작업과 실제 제어 구현 추가는 별도 책임이지만, 빌드 validator는 둘 중 하나가 누락된 상태를 허용하지 않는다.

## pKIRO_rHumanoids 참고 저장소 규칙
- 이 PC의 `/home/kiro/Desktop/pKIRO_rHumanoids`는 실제 로봇 코드를 비교·분석하기 위한 읽기 전용 참고 저장소로 취급한다.
- 일반 작업에서는 해당 저장소의 파일 수정, 커밋, 푸시, 브랜치 변경, 서브모듈 갱신을 수행하지 않는다.
- Sim2Real 반영이 필요한 내용은 `pKIRO_Humanoids`의 관련 코드와 `logs/sim2real/` 지침 문서에 기록하고 Git에 반영한다.
- 사용자가 `pKIRO_rHumanoids` 저장소 자체의 수정을 명시적으로 별도 승인한 경우에만 예외로 작업한다.

## Humanoids/PongBot 구조 동기화 규칙
- `pKIRO_Humanoids`에 적용한 package 구조, 실행 방식, GUI/ROS2 계약, simulator glue 구조는 기본적으로 `pKIRO_PongBotR`에도 같은 변경 단위로 반영한다.
- `pKIRO_PongBotR`이 달라질 수 있는 예외는 로봇 family 고유 제어 로직뿐이다. PongBot은 단순 4족 로봇이므로 `pRbLibrary` core에서 `quadruped_robot_system` 계열을 기준으로 구성하고, Humanoids는 humanoid 계열을 기준으로 구성한다.
- family 고유 제어 로직을 제외한 `mujoco/`, `isaacsim/`, `runtime/`, `threads/`, GUI, app/local GUI 통신, ROS2 topic/message 계약, CMake 구성 방식은 두 패키지에서 동일한 구조와 정책을 유지한다.
- 한 패키지에서 구조 변경을 먼저 적용한 경우 다른 패키지가 뒤처진 구조로 남지 않도록 같은 작업 단위에서 동기 반영한다.
- `pRbLibrary`를 `pKIRO_Humanoids` 기준으로 수정하고 커밋/푸시한 경우, `pKIRO_PongBotR`에서도 해당 `pRbLibrary` 원격 변경을 fetch/pull 또는 submodule update로 반영한 뒤 패키지 서브모듈 포인터를 업데이트한다.
- `pKIRO_PongBotR`에서 `pRbLibrary` 최신 포인터를 반영할 때는 PongBot 고유 설정/YAML과 `quadruped_robot_system` 계약이 깨지지 않는지 빌드와 기본 실행 경로를 확인한다.
- 한쪽 패키지에만 임시 구조, 임시 topic, 임시 CMake path, 임시 simulator wrapper를 추가한 채 방치하지 않는다.

## AGENTS 동기화 규칙
- 워크스페이스 루트 `AGENTS.md`를 기준 원본으로 사용한다.
- 각 로봇 패키지 루트에도 `AGENTS.md` 복제본을 유지한다.
- 루트 `AGENTS.md`가 수정되면 각 패키지 루트 `AGENTS.md`도 동일 내용으로 즉시 동기화한다.
- `pRbLibrary` 하위에는 `AGENTS.md`를 두지 않는다.
- `pRbLibrary` 관련 상세 운영 지침은 각 `pRbLibrary/docs/01. RBLIBRARY_GUIDE.md`와 `pRbLibrary/docs/02. PACKAGE_STRUCTURE.md`에서 관리한다.

## GUI 계약 동기화 규칙
- `pKIRO_Humanoids`와 `pKIRO_PongBotR`의 `RB_GUI`, `RB_GUI_QML`은 GUI 종류와 무관하게 동일한 ROS2 통신 계약을 유지한다.
- 두 패키지의 GUI는 `src/ros2_handler.cpp`와 주고받는 topic, message field, request/response 흐름을 동일하게 유지한다.
- 기준 계약은 최신 구현을 따른다. 현재 GUI task 계약의 기준선은 `request_id`, `task_name`, `/gui_task_ack`, task gating, ack timeout을 포함한 구조다.
- 한 패키지에서 `RB_GUI`, `RB_GUI_QML`, `ros2_handler.cpp`, GUI 관련 message 사용 방식이 바뀌면 다른 패키지와 다른 GUI 타입에도 같은 변경 단위로 동기 반영한다.
- `pKIRO_PongBotR` GUI에서 새 센서 표시, QML component, `QmlBackend`/`GuiBridge` API, sensor topic 계약이 변경되면 `pKIRO_Humanoids` GUI에도 같은 기준으로 반영한다. 반대 방향도 동일하다.
- 한 GUI 타입에만 임시 필드, 임시 topic, 임시 validation, 임시 ack 우회를 추가한 채 방치하지 않는다.
- `RB_GUI`와 `RB_GUI_QML`의 기능 차이가 필요할 때도 ROS2 입출력 계약은 먼저 동일하게 맞추고, UI 표현 차이만 각 GUI 레이어에서 분리한다.
- `RB_GUI_QML`의 QML component 구조, `QmlBackend` property/function surface, `GuiBridge` ROS2 client 계약은 Humanoids와 PongBot에서 동일해야 한다.
- Humanoids에서 만든 QML GUI를 PongBot으로 그대로 옮겨도 실행 가능한 구조를 목표로 하며, 표시되는 기능 차이는 로봇별 YAML/config capability가 결정해야 한다.
- 센서 GUI는 visual-first 원칙을 따른다. `Image`, `PointCloud2`, `NavSatFix`처럼 시각화 가능한 센서는 preview/plot 중심으로 표시하고, `Imu`처럼 비시각 센서는 핵심 값만 compact하게 노출한다.
- sensor topic, message type, frame, source, rate, row 구조 같은 계약 정보는 developer mode에서만 노출한다.
- `config/gui/sensor_topics.yaml` 또는 `RB_GUI_QML/config/sensor_topics.yaml`에 없는 센서, disabled 처리된 센서, visible false 센서는 GUI에 노출하지 않는다.
- task 버튼은 `config/tasks/*.yaml`의 `tasks`, `visible_in_gui`, `allowed_from`, `allowed_to`, `command_mode` 계약으로 생성/비활성화한다.
- manipulation, appendage jog/job, teleop, plot, tuning 같은 GUI 기능은 코드 분기가 아니라 `appendages`, `joint_groups`, `plot_groups`, `gui.capabilities` 같은 설정 계약으로 생성하거나 숨긴다.
- 로봇별로 달라질 수 있는 것은 관절 매핑, gain, model path, appendage 존재 여부, task YAML 내용, 로봇 family 고유 제어 로직뿐이며 GUI 코드 구조 차이로 표현하지 않는다.
- 한 패키지 QML에만 임시 탭, 임시 backend property, 임시 capability 분기를 추가한 채 방치하지 않는다. 기능이 필요하면 공통 backend 계약과 YAML/config capability를 먼저 정의한다.

## 문서 반영 규칙
- 구조, 설정 계약, 운영 원칙, 실행 절차를 수정하면 관련 `md` 문서도 같은 변경 단위에서 함께 갱신한다.
- 최소 반영 대상은 변경 성격에 따라 `README.md`, `pRbLibrary/docs/01. RBLIBRARY_GUIDE.md`, `pRbLibrary/docs/02. PACKAGE_STRUCTURE.md` 중 해당 문서다.
- `pRbLibrary`의 package glue code 경계, config 소유권, appendage/RL 계약이 바뀌면 `pRbLibrary` 문서를 반드시 함께 수정한다.
- 어떤 문서에 반영해야 할지 즉시 판단되지 않으면, 우선 `AGENTS.md`에 운영 규칙으로 기록하고 후속 문서에 승격 반영한다.
