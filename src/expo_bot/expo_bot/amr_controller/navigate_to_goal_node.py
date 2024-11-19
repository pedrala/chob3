import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy

class NavigateToGoalNode(Node):
    def __init__(self):
        super().__init__('navigate_to_goal_node')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # BEST_EFFORT 또는 RELIABLE
            depth=10
        )

        # BasicNavigator 초기화
        self.navigator = BasicNavigator()

        # 초기 위치 설정
        self.set_initial_pose()

        # Nav2 활성화 대기
        self.navigator.waitUntilNav2Active()
        self.get_logger().info("Nav2가 활성화되었습니다.")

        # 목표 좌표 리스트 초기화
        self.target_positions = []

        # 현재 위치 초기화
        self.current_position = Point()

        # 토픽 구독 설정
        self.create_subscription(Point, '/amr_current_position', self.amr_position_callback, qos_profile)
        self.create_subscription(Point, '/target_coordinate', self.target_coordinate_callback, qos_profile)

        # 이동 시작
        self.move_to_next_goal()

    def set_initial_pose(self):
        """초기 위치를 설정"""
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.pose.position.x = 0.0
        initial_pose.pose.position.y = 0.0

        # 오일러 각도 -> 쿼터니안 변환
        roll, pitch, yaw = 0.0, 0.0, 0.1178  # 예: 6.75도
        quat = self.euler_to_quaternion(roll, pitch, yaw)
        initial_pose.pose.orientation.x = quat[0]
        initial_pose.pose.orientation.y = quat[1]
        initial_pose.pose.orientation.z = quat[2]
        initial_pose.pose.orientation.w = quat[3]

        self.navigator.setInitialPose(initial_pose)
        self.get_logger().info("초기 위치가 설정되었습니다.")

    def amr_position_callback(self, msg):
        """AMR의 현재 위치를 업데이트"""
        self.current_position = msg
        self.get_logger().info(f"AMR 현재 위치 업데이트: x={msg.x}, y={msg.y}")

    def target_coordinate_callback(self, msg):
        """목표 좌표를 수신하여 리스트에 추가"""
        self.target_positions.append(msg)
        self.get_logger().info(f"목표 좌표 수신: x={msg.x}, y={msg.y}")

    def move_to_next_goal(self):
        """목표 좌표로 이동"""
        if not self.target_positions:
            self.get_logger().info("이동할 목표 좌표가 없습니다.")
            return

        target = self.target_positions.pop(0)
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = target.x
        goal_pose.pose.position.y = target.y

        # 방향 설정
        roll, pitch, yaw = 0.0, 0.0, 0.0
        quat = self.euler_to_quaternion(roll, pitch, yaw)
        goal_pose.pose.orientation.x = quat[0]
        goal_pose.pose.orientation.y = quat[1]
        goal_pose.pose.orientation.z = quat[2]
        goal_pose.pose.orientation.w = quat[3]

        self.get_logger().info(f"목표 좌표로 이동: x={target.x}, y={target.y}")
        self.navigator.goToPose(goal_pose)

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                distance_to_goal = math.sqrt(
                    (goal_pose.pose.position.x - self.current_position.x) ** 2 +
                    (goal_pose.pose.position.y - self.current_position.y) ** 2
                )
                self.get_logger().info(f"남은 거리: {distance_to_goal:.2f}m")
                if distance_to_goal < 0.2:
                    self.get_logger().info("목표에 도달했습니다.")
                    break

        result = self.navigator.getResult()
        if result == 1:  # 성공
            self.get_logger().info(f"목표 도달 완료: x={target.x}, y={target.y}")
        else:
            self.get_logger().warn(f"목표 도달 실패: x={target.x}, y={target.y}")

        # 다음 목표 이동
        self.move_to_next_goal()

    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        """오일러 각도를 쿼터니안으로 변환"""
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return qx, qy, qz, qw


def main():
    rclpy.init()
    node = NavigateToGoalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
