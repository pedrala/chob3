import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator


""" 1. 맵 프레임과 초기 위치 설정
        set_initial_pose에서 맵 프레임(map)과 로봇의 초기 위치를 정확히 설정해야 합니다.
        맵 서버가 실행 중이어야 하고, 로봇이 맵 내에서 정확히 로컬라이즈 되어 있어야 합니다.
        로컬라이제이션이 제대로 설정되지 않으면 Nav2가 경로를 생성하지 못합니다.
        확실히 초기 위치를 설정한 후 Nav2가 활성화되었는지 확인합니다.

    2. 맵 서버와 로컬라이제이션 실행
        터틀봇이 맵 서버와 로컬라이제이션 노드를 통해 맵 상의 위치를 인식해야 합니다. 아래 명령어를 확인하세요.

        # 맵 서버 실행
        ros2 run nav2_map_server map_server --ros-args -p yaml_filename:="<맵 파일 경로>"

        # 로컬라이제이션 실행 (예: AMCL)
        ros2 launch nav2_bringup localization_launch.py use_sim_time:=false map:="<맵 파일 경로>"

    3. 목표 좌표의 프레임 확인
        목표 좌표(goal_pose.header.frame_id)의 프레임이 map으로 설정되어야 합니다. 

    4. AMR 로봇과 Nav2 구성 확인
        로봇의 위치(/amr_current_position)가 맵 상의 좌표계(map)에 정확히 매핑되었는지 확인하세요.
        Nav2 설정 파일(nav2_params.yaml)에서 controller_server, planner_server 등의 플러그인 파라미터가 적절한지 확인합니다.

        yaml

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
 """


# 목표로 이동하며 AMR 현재 위치를 확인하는 노드
class NavigateToGoalNode(Node):
    def __init__(self):
        super().__init__('navigate_to_goal_node')
        
        # BasicNavigator 인스턴스 초기화
        self.navigator = BasicNavigator()

        # 초기 위치 설정
        self.set_initial_pose()

        # Nav2 활성화 대기
        self.navigator.waitUntilNav2Active()

        # /target_coordinate 토픽 구독 (목표 좌표 수신)
        self.create_subscription(Point, '/target_coordinate', self.target_callback, 10)
        
        # /amr_current_position 토픽 구독 (현재 위치 수신)
        self.create_subscription(Point, '/amr_current_position', self.amr_position_callback, 10)
        
        # AMR 로봇의 현재 위치를 저장할 변수
        self.current_position = Point()

    # 초기 위치를 설정하는 함수
    def set_initial_pose(self):
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'  # 맵 프레임
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.pose.position.x = 0.0  # 초기 x 위치
        initial_pose.pose.position.y = 0.0  # 초기 y 위치
        initial_pose.pose.orientation.z = 0.0  # 초기 방향
        initial_pose.pose.orientation.w = 1.0  # 초기 방향

        self.navigator.setInitialPose(initial_pose)
        self.get_logger().info("초기 위치가 설정되었습니다.")

        # 로봇이 로컬라이제이션을 수행할 시간을 기다림
        self.get_logger().info("Nav2 초기화 및 로컬라이제이션 대기 중...")
        rclpy.sleep(5)

    # AMR 현재 위치를 수신하여 변수에 저장하는 콜백 함수
    def amr_position_callback(self, msg):
        self.current_position = msg
        self.get_logger().info(f"AMR 현재 위치 수신: x={msg.x}, y={msg.y}")

    # 목표 좌표 수신 콜백 함수
    def target_callback(self, msg):
        # 목표 좌표 설정
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = msg.x
        goal_pose.pose.position.y = msg.y
        goal_pose.pose.orientation.w = 1.0  # 기본 방향 설정 (회전 없이 직진)
        # 이동 시작 알림 및 목표로 이동
        self.get_logger().info("목표 좌표 수신 - 이동 시작")
        self.navigator.goToPose(goal_pose)
        # 이동 완료 여부 확인 및 현재 위치와의 거리 출력
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                distance_to_goal = ((goal_pose.pose.position.x - self.current_position.x) ** 2 +
                                    (goal_pose.pose.position.y - self.current_position.y) ** 2) ** 0.5
                self.get_logger().info(f"목표까지 남은 거리: {distance_to_goal:.2f}m")

        # 이동 결과 확인
        result = self.navigator.getResult()
        if result == BasicNavigator.Result.SUCCEEDED:
            self.get_logger().info("목표 좌표에 도착했습니다.")
            # 이동이 성공하면 다음 목표 좌표를 처리
            self.move_to_next_goal()
        else:
            self.get_logger().info("목표 좌표에 도착하지 못했습니다.")

    def move_to_next_goal(self):
        # 목표 좌표들 중 다음 좌표로 이동 (여기서는 목표 좌표 목록을 받아오는 방식으로 구현)
        self.get_logger().info("다음 목표 좌표로 이동 중...")
        # 예시: target_positions라는 리스트에서 목표 좌표를 순차적으로 받아오는 코드 추가
        if len(self.target_positions) > 0:
            next_goal = self.target_positions.pop(0)
            self.target_callback(next_goal)
        else:
            self.get_logger().info("남은 목표 좌표가 없습니다.")

# ROS2 노드 실행 함수
def main():
    rclpy.init()
    node = NavigateToGoalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
