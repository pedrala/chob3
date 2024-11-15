This project is for implemeting amr service robot promoting company in the convention.

#패키지 빌드

colcon build --packages-select expo_bot

#실행하는 순서

1. ros2 run expo_bot cctv_detection_node my_best.pt #cctv 관제 시스템 가동 시작, 압축하여 이미지 전송

2. ros2 run expo_bot amr_control_node #AMR봇 자체 좌표 및 영상 토픽으로 발행

3. ros2 run expo_bot navigate_to_goal_node #임의의 정해진 목표로 네비게이션 

4. ros2 run expo_bot sys_monitor_node #모니터링 시스템에서 영상 디스플레이, 구독받은 객체정보 DB 에 저장
