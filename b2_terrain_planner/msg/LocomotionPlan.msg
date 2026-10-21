# TAMOLS 지형 플래너가 발행하는 보행 궤적. WBC 추종 모드가 소비한다.
# 몸통 궤적과 네 발 궤적을 공통 시간격자(dt)로 샘플링해 담는다.
std_msgs/Header header          # stamp=계획 시각, frame_id=odom
float64 dt                      # 샘플 간격 [s]
uint32 num_samples              # 시간 샘플 수 N (= horizon/dt + 1)

# 샘플별 N개. 필드명은 기존 wire 계약을 유지하지만 병진은 전신 CoM이다.
# base_pose.position을 base 원점 위치로 해석하면 안 된다.
geometry_msgs/Pose[] base_pose    # 전신 CoM 위치 + 몸통 자세(quaternion), odom
geometry_msgs/Twist[] base_twist  # CoM 월드 선속도 + 몸통 Euler 변화율(roll,pitch,yaw)
# 수신 bridge는 angular를 월드 각속도로 변환한다(euler_rates 설정).

# 발 궤적: 4개 다리 × N 샘플, FL,FR,RL,RR 순으로 평탄화(길이 4*N).
geometry_msgs/Point[] foot_position  # odom 프레임 발끝 위치
bool[] foot_contact                  # true=접지(stance), false=스윙

# 발 순서 상수.
uint8 FL=0
uint8 FR=1
uint8 RL=2
uint8 RR=3

# WBC 소유 보행 시간. ROS 시각(header)은 신선도 검사에 계속 사용한다.
bool gait_clock_valid
int64 gait_epoch_ns
float64 gait_time
# 고정 trot 위상: FL/RR=0, FR/RL=0.5. 아래 값으로 접촉 경계를 정확히 평가.
float64 gait_period
float64 gait_duty

# 갱신은 이 부모 계획을 추종 중일 때만 채택한다(최초 계획은 0).
int64 parent_plan_stamp_ns
# 이 보행 시각까지 몸통은 부모 궤적 그대로이며 이후 새 최적화 궤적으로 이어진다.
float64 handover_gait_time
# 몸통은 별도 시간으로 평가한다. gait_duration은 이 계획의 유효 발 위상 구간이다.
# hold_gait 계획은 확인 지지를 유지하며 다음 이륙을 허용하지 않는다.
bool body_clock_valid
float64 body_time
float64 handover_body_time
# 보행 재개 예정 시각. 재계획마다 뒤로 밀지 않는다.
float64 gait_resume_body_time
float64 gait_duration
bool hold_gait
# 몸통 다항식: 각 구간의 시작은 body_time(legacy는 gait_time) 기준 상대초, 마지막 원소는 horizon.
# 계수는 구간별 [CoM x,y,z,yaw,pitch,roll][power 0..4], 물리 초 단위.
# 빈 배열은 기존 sampled 메시지와의 호환 경로다.
float64[] body_breaks
float64[] body_coefficients
