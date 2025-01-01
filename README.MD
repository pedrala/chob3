## Description

컨벤션센터의 CCTV에서 구독받은 이미지를 Flask를 통해 웹화면에 보여주고 시스템 모니터 노드에서는 YOLO 객체인식을 처리합니다. 화면의 바운딩박스안에 3명 이상이 모였다고 판단한 순간 목표좌표를 AMR 봇에게 전달하면 AMR봇은 해당위치로 이동합니다. 이동중 카메라의 이미지를 구독받아 웹화면에 디스플레이합니다.

## Demo

<p align="center">
  <img src="/expobot_demo.png" alt="expobot_demo" width="1000">
</p>


## 패키지 빌드

```console
cd ~/chob3_ws
colcon build or
colcon build --packages-select expo_bot

source install/setup.bash
```
## 실행하는 순서

### 1.cctv yolo 객체 인식 시작, 이미지 압축하여 발행, 객체인식한 정보 발행

```console
ros2 run expo_bot cctv_detection_node cctv.pt 
```

### 2. 모니터링 시스템에서 cctv이미지영상, 로봇캠 영상 구독받아 디스플레이, 목표좌표 퍼블리쉬, 구독받은 객체정보 DB 에 저장

```console
ros2 run expo_bot sys_monitor_node  
```

### 3. AMR봇 자체 좌표 및 영상 토픽으로 발행 (AMR 에 원격 접속하여 해당폴더로 이동하여 실행할 것)

```console
ros2 run expo_bot amr_control_node 
```

### 4. 위치 초기화, 목표좌표(JSON))와 AMR봇좌표를 구독받고  목표좌표로 네비게이션 (AMR 에 원격 접속하여 해당폴더로 이동하여 실행할 것)

```console
ros2 run expo_bot navigate_to_goal_node 
```
