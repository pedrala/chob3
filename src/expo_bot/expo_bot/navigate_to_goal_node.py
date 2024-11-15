import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator

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
        initial_pose.header.frame_id = 'map'
        initial_pose.pose.position.x = 0.0  # 초기 x 위치
        initial_pose.pose.position.y = 0.0  # 초기 y 위치
        initial_pose.pose.orientation.z = 0.0  # 초기 방향 (쿼터니언 z)
        initial_pose.pose.orientation.w = 1.0  # 초기 방향 (쿼터니언 w)

        self.navigator.setInitialPose(initial_pose)
        self.get_logger().info("초기 위치가 설정되었습니다.")

    # AMR 현재 위치를 수신하여 변수에 저장하는 콜백 함수
    def amr_position_callback(self, msg):
        self.current_position = msg
        self.get_logger().info(f"AMR 현재 위치 수신: x={msg.x}, y={msg.y}")

    # 목표 좌표 수신 콜백 함수
    def target_callback(self, msg):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x = msg.x
        goal_pose.pose.position.y = msg.y
        goal_pose.pose.orientation.w = 1.0

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
            #다음 목표로 이동
            # for range : 

        else:
            self.get_logger().info("목표 좌표에 도착하지 못했습니다.")

# ROS2 노드 실행 함수
def main():
    rclpy.init()
    node = NavigateToGoalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
