# WBC의 동일 제어 프레임 상태. 실기/시뮬레이션 공통 센서·추정기만 사용한다.
# header.stamp는 제어 시각이며 발행 시각으로 덮어쓰지 않는다.
# 위치와 모든 속도는 header.frame_id(연속 KF odom) 기준, SI 단위.
# base_orientation은 body -> odom 회전. angular_velocity는 Euler 변화율이 아니다.
# CoM/발: KF base + IMU + 측정 관절 q/dq의 전신 운동학 결과.
# 발 순서 FL, FR, RL, RR. contact_estimated는 모멘텀 관측기 기반 추정이며
# 시뮬레이터 접촉 정답이나 계획된 접촉 상태가 아니다.
# 필요한 상태가 유효한 프레임만 발행한다. 수신자는 시각 만료를 검사해야 한다.
std_msgs/Header header
geometry_msgs/Point base_position
geometry_msgs/Point com_position
geometry_msgs/Vector3 com_velocity
geometry_msgs/Quaternion base_orientation
geometry_msgs/Vector3 base_angular_velocity
geometry_msgs/Point[4] foot_position
geometry_msgs/Vector3[4] foot_velocity
bool[4] contact_estimated

# WBC 소유 보행 시간. ROS 시각(header)은 신선도 검사에 계속 사용한다.
bool gait_clock_valid
int64 gait_epoch_ns
float64 gait_time  # 이번 상태에서 실제 평가한 위상 시각(다음 tick 시각 아님)
bool gait_paused
# 몸통 시간은 착지 대기에도 진행한다. 측정 상태와 같은 제어 tick의 시각이다.
bool body_clock_valid
float64 body_time
bool[4] contact_confirmed

# 제어 주기에서 실제 채택한 계획. 수신 mailbox stamp와 구분한다.
int64 active_plan_stamp_ns
# 가장 최근 제어 주기 후보 판정(0 없음, 1 채택, 2 부모, 3 연결시각, 4 무효,
# 5 epoch, 6 연속성). 실행 계획 stamp와 독립적이다.
int64 candidate_plan_stamp_ns
int32 plan_adoption_status
