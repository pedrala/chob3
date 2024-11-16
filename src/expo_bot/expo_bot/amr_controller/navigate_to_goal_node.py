
""" 
    ssh -X rokey9@192.168.10.56

    ros2 launch turtlebot3_bringup robot.launch.py

    1. 맵 프레임과 초기 위치 설정
        set_initial_pose에서 맵 프레임(map)과 로봇의 초기 위치를 정확히 설정해야 합니다.
        맵 서버가 실행 중이어야 하고, 로봇이 맵 내에서 정확히 로컬라이즈 되어 있어야 합니다.
        로컬라이제이션이 제대로 설정되지 않으면 Nav2가 경로를 생성하지 못합니다.
        확실히 초기 위치를 설정한 후 Nav2가 활성화되었는지 확인합니다.

    2. 맵 서버와 로컬라이제이션 실행
        터틀봇이 맵 서버와 로컬라이제이션 노드를 통해 맵 상의 위치를 인식해야 합니다. 아래 명령어를 확인하세요.

        # 맵 서버 실행
        ros2 run nav2_map_server map_server --ros-args -p yaml_filename:="/home/rokey9/chob3/tb_ws_map.yaml"

        
        # 로컬라이제이션 실행 (예: AMCL)
        ros2 launch nav2_bringup localization_launch.py use_sim_time:=false map:="/home/rokey9/chob3/tb_ws_map.yaml"
        
        

    3. 목표 좌표의 프레임 확인
        목표 좌표(goal_pose.header.frame_id)의 프레임이 map으로 설정되어야 합니다. 

    4. AMR 로봇과 Nav2 구성 확인
        로봇의 위치(/amr_current_position)가 맵 상의 좌표계(map)에 정확히 매핑되었는지 확인하세요.
        Nav2 설정 파일(nav2_params.yaml)에서 controller_server, planner_server 등의 플러그인 파라미터가 적절한지 확인합니다.

        yaml

        image: tb_ws_map.pgm  # 맵 이미지 파일 경로
        resolution: 0.05      # 맵의 해상도 (m/pixel)
        origin: [0.0, 0.0, 0.0] # 맵의 원점 좌표
        negate: 0             # 이미지 반전 여부 (0: 안 함)
        occupied_thresh: 0.65 # 점유 임계값
        free_thresh: 0.196    # 비점유 임계값

        planner_server:
        expected_planner_frequency: 5.0
        controller_server:
        expected_controller_frequency: 10.0
        default_plugin: "FollowPath"
        behavior_server:
        default_behavior_tree: "navigate_w_replanning_time.xml"

    5. 오도메트리 및 TF 변환 확인
        로봇이 맵 좌표계(map)와 기본 좌표계(odom, base_link) 간의 TF를 올바르게 전송하고 있는지 확인하세요:

        ros2 run tf2_ros tf2_echo map base_link
        로봇의 위치가 올바르게 표시되지 않으면 TF 설정이 잘못되었을 가능성이 있습니다.
    6. 움직이지 않을 때 디버깅
        로봇이 움직이지 않는 이유를 디버깅하기 위해 로그를 확인합니다:

        # Nav2 상태 확인
        ros2 topic echo /amcl_pose
        ros2 topic echo /tf
        ros2 topic echo /goal_pose
        경로가 생성되지 않았다면 플래너의 문제가 있을 수 있습니다.
        로봇이 경로를 따라가지 않는다면 컨트롤러 또는 로컬라이제이션 문제가 있을 수 있습니다.

    7. 예제 목표 좌표 설정 및 실행
        목표 좌표를 설정하고 제대로 동작하는지 테스트합니다:

        ros2 topic pub /target_coordinate geometry_msgs/Point "{x: 2.0, y: 3.0, z: 0.0}"

    8. 완전한 Nav2 실행 명령
        터틀봇이 제대로 동작하지 않으면 아래와 같이 전체 Nav2를 실행하며 테스트합니다:

        ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false
        
        결론:
        맵 서버와 로컬라이제이션 노드를 실행하여 터틀봇이 맵 상의 위치를 인식할 수 있도록 설정합니다.
        초기 위치 및 목표 좌표의 프레임을 올바르게 설정합니다.
        Nav2의 설정과 로봇의 TF를 확인하여 문제가 없는지 확인하세요.


        

    종합적인 명령어 실행 예시
        TurtleBot3을 시작하고, Navigation2 관련 패키지를 실행하는 전체 과정은 다음과 같습니다:

        로봇 기본 런치:

        ros2 launch turtlebot3_bringup robot.launch.py

        맵 서버 실행:

        ros2 run nav2_map_server map_server --ros-args -p yaml_filename:="/home/rokey9/chob3/tb_ws_map.yaml"

        AMCL 실행:

        ros2 run nav2_amcl amcl --ros-args -p map_frame:="map" -p base_frame:="base_link" -p odom_frame:="odom"

        로봇 상태 퍼블리셔 실행:

        ros2 run robot_state_publisher robot_state_publisher

        경로 계획 및 제어 노드 실행:

        ros2 run nav2_planner planner_server
        ros2 run nav2_controller controller_server

        목표 지점 설정 (예시):

        ros2 topic pub /goal geometry_msgs/msg/PoseStamped "header:
        frame_id: 'map'
        pose:
        position:
            x: 1.0
            y: 1.0
            z: 0.0
        orientation:
            x: 0.0
            y: 0.0
            z: 0.0
            w: 1.0"

            

        이 과정을 통해 TurtleBot3은 주어진 맵에서 지정된 목표 지점으로 이동할 수 있습니다.


 
 """

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator

class NavigateToGoalNode(Node):
    def __init__(self):
        super().__init__('navigate_to_goal_node')
        
        # BasicNavigator 초기화
        self.navigator = BasicNavigator()

        # 초기 위치 설정
        self.set_initial_pose()

        # Nav2 활성화 대기
        self.navigator.waitUntilNav2Active()
        self.get_logger().info("Nav2가 활성화되었습니다.")

        # 목표 좌표와 현재 좌표 초기화
        self.target_positions = []  # 목표 좌표 리스트
        self.current_position = Point()  # AMR 현재 위치 저장

        # /target_coordinate 토픽 구독 (목표 좌표 수신)
        self.create_subscription(Point, '/target_coordinate', self.target_callback, 10)

        # /amr_current_position 토픽 구독 (현재 위치 수신)
        self.create_subscription(Point, '/amr_current_position', self.amr_position_callback, 10)

    def set_initial_pose(self):
        """맵 프레임에서 초기 위치 설정"""
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'  # 맵 좌표계
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.pose.position.x = 0.0
        initial_pose.pose.position.y = 0.0
        initial_pose.pose.orientation.z = 0.0
        initial_pose.pose.orientation.w = 1.0

        self.navigator.setInitialPose(initial_pose)
        self.get_logger().info("초기 위치가 설정되었습니다.")

    def amr_position_callback(self, msg):
        """AMR 현재 위치를 업데이트"""
        self.current_position = msg
        self.get_logger().info(f"AMR 현재 위치: x={msg.x}, y={msg.y}")

    def target_callback(self, msg):
        """목표 좌표를 수신하고 리스트에 추가"""
        self.get_logger().info(f"새로운 목표 좌표 수신: x={msg.x}, y={msg.y}")
        self.target_positions.append(msg)
        if len(self.target_positions) == 1:
            self.move_to_next_goal()  # 첫 목표 좌표로 즉시 이동

    def move_to_next_goal(self):
        """목표 좌표 리스트에서 다음 위치로 이동"""
        if len(self.target_positions) == 0:
            self.get_logger().info("이동할 목표 좌표가 없습니다.")
            return

        # 목표 좌표 설정
        target = self.target_positions.pop(0)
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = target.x
        goal_pose.pose.position.y = target.y
        goal_pose.pose.orientation.w = 1.0  # 기본 방향 설정

        self.get_logger().info(f"목표 좌표로 이동 시작: x={target.x}, y={target.y}")
        self.navigator.goToPose(goal_pose)

        # 이동 완료 여부 확인
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                distance_to_goal = ((goal_pose.pose.position.x - self.current_position.x) ** 2 +
                                    (goal_pose.pose.position.y - self.current_position.y) ** 2) ** 0.5
                self.get_logger().info(f"목표까지 남은 거리: {distance_to_goal:.2f}m")

        # 이동 결과 확인
        result = self.navigator.getResult()
        if result == BasicNavigator.Result.SUCCEEDED:
            self.get_logger().info(f"목표 좌표에 도착했습니다: x={target.x}, y={target.y}")
        else:
            self.get_logger().info("목표 좌표에 도착하지 못했습니다.")

        # 다음 목표 좌표로 이동
        self.move_to_next_goal()

def main():
    rclpy.init()
    node = NavigateToGoalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
