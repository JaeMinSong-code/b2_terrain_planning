# B2 모델 출처

- 원본 저장소: `/home/js/ros2_ws/src/b2_deploy_ws`
- 기준 커밋: `3d887711c58799f8a944046f3ba2ea221441aa26`
- 원본: `src/pRbLibrary/model/unitree_b2/b2.urdf`
- 복제본: `unitree_b2/b2.urdf`
- SHA-256: `ddb9a1e4505204a40c3b7dca06b362c89f5a98fb7c1ef1517b37505425c31d26`

WBC의 `config/robots/unitree_b2.yaml`이 선택하는 URDF를 내용 변경 없이 복제했다.
pRbLibrary는 해당 저장소에서 일반 파일로 관리한다. 이 패키지는 독립 배포를 위해 URDF를
소유하며 원본을 자동 동기화하지 않는다. 원본을 갱신할 때 기준 커밋과 해시를 함께 갱신하고
모델 FK·관성·IK와 계획 입출력의 일관성을 확인한다.
현재 실행 경로는 `tamols_solve`와 `terrain_planner_node`이며 별도 모델 시험 도구는 포함하지 않는다.

기준 관절 자세는 `config/profiles/job/home.yaml`의 다리별 `[0, 44.30, -85.94]`도다.
원본 base z=0.45 m는 평지 계획 예제에서 그대로 사용하지 않는다. FK 발 원점을 지면에
맞추는 전체 수직 이동을 명시적으로 적용하여 base z≈0.512059497 m로 예제를 구성했다.
관절 순서는 FL/FR/RL/RR × hip/thigh/calf이며 발 위치는 `*_foot` 링크 원점이다.

메시는 복제하지 않았다. RBDL의 기구학·관성 계산용이며 이 복제본만으로 시각화나
충돌 메시 로딩을 지원하지 않는다. 모델 값은 URDF 기준이며 실제 하드웨어 실측 보정의
완료를 의미하지 않는다. 원본 저장소의 다른 세션 변경은 수정하지 않았다.
