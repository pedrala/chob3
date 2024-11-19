This project aims to implement an AMR service robot to promote a company at an exhibition.

## 패키지 빌드
--------------------------------------------------------------------

```console
colcon build
colcon build --packages-select expo_bot

source install/setup.bash
```
## 실행하는 순서
--------------------------------------------------------------------

### 1.cctv yolo 객체 인식 시작, 이미지 압축하여 발행, 객체인식한 정보 발행

```console
ros2 run expo_bot cctv_detection_node cctv.pt 
```

### 2. 모니터링 시스템에서 cctv이미지영상, 로봇캠 영상 구독받아 디스플레이, 목표좌표 퍼블리쉬, 구독받은 객체정보 DB 에 저장

```console
ros2 run expo_bot sys_monitor_node  
```

### 3. AMR봇 자체 좌표 및 영상 토픽으로 발행, 

```console
ros2 run expo_bot amr_control_node 
```

### 4. 위치 초기화, 목표좌표(JSON))와 AMR봇좌표를 구독받고  목표좌표로 네비게이션 

```console
ros2 run expo_bot navigate_to_goal_node 
```
