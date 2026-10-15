# TAMOLS 지형 플래너가 발행하는 보행 궤적. WBC 추종 모드가 소비한다.
# 몸통 궤적과 네 발 궤적을 공통 시간격자(dt)로 샘플링해 담는다.
std_msgs/Header header          # stamp=계획 시각, frame_id=odom
float64 dt                      # 샘플 간격 [s]
uint32 num_samples              # 시간 샘플 수 N (= horizon/dt + 1)

# 몸통 궤적: 샘플별 N개.
geometry_msgs/Pose[] base_pose    # 위치 + 자세(quaternion), odom 프레임
geometry_msgs/Twist[] base_twist  # 월드 선속도 + 월드 각속도

# 발 궤적: 4개 다리 × N 샘플, FL,FR,RL,RR 순으로 평탄화(길이 4*N).
geometry_msgs/Point[] foot_position  # odom 프레임 발끝 위치
bool[] foot_contact                  # true=접지(stance), false=스윙

# 발 순서 상수.
uint8 FL=0
uint8 FR=1
uint8 RL=2
uint8 RR=3
