import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import json
import math

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

        # /amr_current_position 토픽 구독 (현재 위치 수신)
        self.create_subscription(Point, '/amr_current_position', self.amr_position_callback, 10)

        # goal_position.json 파일에서 목표 좌표를 읽어 리스트에 저장
        self.get_target_positions_from_file()

        # 목표 좌표로 이동
        self.move_to_next_goal()

        # 목표 좌표 발행을 위한 퍼블리셔 생성
        self.goal_publisher = self.create_publisher(PoseStamped, '/move_base_simple/goal', 10)

    def set_initial_pose(self):
        """맵 프레임에서 초기 위치 설정"""
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'  # 맵 좌표계
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.pose.position.x = 0.0
        initial_pose.pose.position.y = 0.0
        
        # 오일러 각도 -> 쿼터니안 변환
        roll = 0.0
        pitch = 0.0
        yaw = 0.1178  # 약 6.75도 (라디안 값)
        quat = self.euler_to_quaternion(roll, pitch, yaw)
        initial_pose.pose.orientation.x = quat[0]
        initial_pose.pose.orientation.y = quat[1]
        initial_pose.pose.orientation.z = quat[2]
        initial_pose.pose.orientation.w = quat[3]

        self.navigator.setInitialPose(initial_pose)
        self.get_logger().info("초기 위치가 설정되었습니다.")

    def get_target_positions_from_file(self):
        """goal_position.json 파일에서 목표 좌표 가져오기"""
        file_path = './src/expo_bot/resource/goal_position.json'
        try:
            with open(file_path, "r", encoding='utf-8') as file:
                json_data = json.load(file)
                
                if len(json_data) >= 1:
                    # 최근 5개 목표 좌표 가져오기
                    for target_data in json_data[-5:]:
                        position = target_data['pose']['pose']['position']
                        target_position = Point(
                            x=position['x'],
                            y=position['y'],
                            z=position['z']
                        )
                        self.target_positions.append(target_position)

                    self.get_logger().info(f"목표 좌표가 {len(self.target_positions)}개 로드되었습니다.")
                else:
                    self.get_logger().warn(f"goal_position.json에 데이터가 충분하지 않습니다.")
        except Exception as e:
            self.get_logger().error(f"파일을 읽는 중 오류 발생: {e}")

    def amr_position_callback(self, msg):
        """AMR 현재 위치를 업데이트"""
        self.current_position = msg
        self.get_logger().info(f"AMR 현재 위치: x={msg.x}, y={msg.y}")

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

        # 방향 설정 (오일러 -> 쿼터니안)
        roll = 0.0
        pitch = 0.0
        yaw = 0.0  # 목표 지점에서의 방향 (0으로 기본값 설정)
        quat = self.euler_to_quaternion(roll, pitch, yaw)
        goal_pose.pose.orientation.x = quat[0]
        goal_pose.pose.orientation.y = quat[1]
        goal_pose.pose.orientation.z = quat[2]
        goal_pose.pose.orientation.w = quat[3]

        self.get_logger().info(f"목표 좌표로 이동 시작: x={target.x}, y={target.y}")

        # 목표 좌표를 /move_base_simple/goal 토픽에 발행
        self.goal_publisher.publish(goal_pose)
        self.get_logger().info(f"목표 좌표 발행: x={target.x}, y={target.y}")

        self.navigator.goToPose(goal_pose)

        # 이동 완료 여부 확인
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                distance_to_goal = ((goal_pose.pose.position.x - self.current_position.x) ** 2 +
                                    (goal_pose.pose.position.y - self.current_position.y) ** 2) ** 0.5
                self.get_logger().info(f"목표까지 남은 거리: {distance_to_goal:.2f}m")
                if distance_to_goal < 0.2:  # 예시로 20cm 이하로 가까워졌을 때
                    self.get_logger().info("목표에 충분히 근접하여 이동을 종료합니다.")
                    break

        # 이동 결과 확인
        result = self.navigator.getResult()
        if result == 1:  # 기본값 1이 성공을 의미
            self.get_logger().info(f"목표 좌표에 도달했습니다: x={target.x}, y={target.y}")
        else:
            self.get_logger().info("목표 좌표에 도달하지 못했습니다.")

        # 다음 목표 좌표로 이동
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
