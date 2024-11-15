This project is for implemeting amr service robot promoting company in the convention.

#패키지 빌드

colcon build --packages-select expo_bot

#실행하는 순서

#cctv yolo 객체 인식 시작, 이미지 압축하여 발행, 객체인식한 정보 발행, 목표좌표 발행
1. ros2 run expo_bot cctv_detection_node my_best.pt 

#AMR봇 자체 좌표 및 영상 토픽으로 발행
2. ros2 run expo_bot amr_control_node 

#목표좌표(리스트)와 AMR봇좌표를 구독받고 목표좌표로 네비게이션 
3. ros2 run expo_bot navigate_to_goal_node 

#모니터링 시스템에서 영상 디스플레이, 구독받은 객체정보 DB 에 저장
4. ros2 run expo_bot sys_monitor_node 
