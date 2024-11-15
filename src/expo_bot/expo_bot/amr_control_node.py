import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import random  # 예시용 랜덤 좌표 생성
import time

# AMR 로봇의 현재 위치를 퍼블리시하는 노드
class AMRControlNode(Node):
    def __init__(self):
        super().__init__('amr_control_node')
        
        # 현재 위치를 발행할 토픽 생성
        self.position_publisher_ = self.create_publisher(Point, '/amr_current_position', 10)
        
        # 일정 주기로 위치 퍼블리시 (초 단위로 설정 가능)
        self.timer_ = self.create_timer(1.0, self.publish_position)

    # 현재 좌표를 퍼블리시하는 함수
    def publish_position(self):
        # 예시용으로 랜덤 위치 설정 (실제로는 로봇의 위치 정보를 사용)
        current_position = Point()
        current_position.x = random.uniform(0.0, 5.0)
        current_position.y = random.uniform(0.0, 5.0)
        current_position.z = 0.0  # 2D 위치
        
        # 좌표를 퍼블리시
        self.position_publisher_.publish(current_position)
        self.get_logger().info(f"AMR 현재 위치 발행: x={current_position.x}, y={current_position.y}")

# ROS2 노드 실행 함수
def main():
    rclpy.init()
    node = AMRControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
