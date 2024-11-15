import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image  # 이미지를 퍼블리시하기 위해 필요한 메시지
from cv_bridge import CvBridge  # OpenCV 이미지를 ROS2 이미지 메시지로 변환
import random  # 예시용 랜덤 좌표 생성
import cv2  # OpenCV
import time

class AMRControlNode(Node):
    def __init__(self):
        super().__init__('amr_control_node')
        
        # 현재 위치를 발행할 토픽 생성
        self.position_publisher_ = self.create_publisher(Point, '/amr_current_position', 10)
        
        # 카메라 영상을 발행할 토픽 생성
        self.image_publisher_ = self.create_publisher(Image, '/amr_camera_image', 10)
        
        # OpenCV 브리지 생성
        self.bridge = CvBridge()

        # 카메라 초기화 (예시로 로컬 카메라 사용)
        self.cap = cv2.VideoCapture(0)
        
        # 일정 주기로 위치와 영상을 퍼블리시 (초 단위로 설정 가능)
        self.timer_ = self.create_timer(1.0, self.publish_data)

    # 현재 좌표와 영상을 퍼블리시하는 함수
    def publish_data(self):
        # 예시용으로 랜덤 위치 설정 (실제로는 로봇의 위치 정보를 사용)
        current_position = Point()
        current_position.x = random.uniform(0.0, 5.0)
        current_position.y = random.uniform(0.0, 5.0)
        current_position.z = 0.0  # 2D 위치
        
        # 좌표를 퍼블리시
        self.position_publisher_.publish(current_position)
        self.get_logger().info(f"AMR 현재 위치 발행: x={current_position.x}, y={current_position.y}")

        # 카메라에서 프레임 읽기
        ret, frame = self.cap.read()
        if ret:
            # OpenCV 이미지를 ROS2 이미지 메시지로 변환
            image_message = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            
            # 이미지를 퍼블리시
            self.image_publisher_.publish(image_message)
            self.get_logger().info("AMR 카메라 영상 발행")

    def destroy_node(self):
        # 카메라 자원 해제
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

# ROS2 노드 실행 함수
def main():
    rclpy.init()
    node = AMRControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
