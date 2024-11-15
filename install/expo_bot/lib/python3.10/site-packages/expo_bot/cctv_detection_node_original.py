import cv2
from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Int32

class CCTVCrowdDetectionNode(Node):
    def __init__(self):
        super().__init__('cctv_detection_node')
        self.publisher_ = self.create_publisher(Point, '/crowd_location', 10)

        self.crowd_threshold = 5  # 군중 인식 임계치 설정
        self.timer = self.create_timer(1.0, self.detect_crowd)  # 1초마다 군중 탐지 실행

    def detect_crowd(self):
        # CCTV로부터 군중 데이터 획득 (샘플 데이터 사용)
        detected_people_count = 7  # 샘플 값, 실제 코드에서는 CCTV 영상 분석 필요
        
        # 군중이 임계치를 넘으면 위치 정보 발행
        if detected_people_count >= self.crowd_threshold:
            point = Point(x=1.0, y=2.0, z=0.0)  # 샘플 좌표
            self.publisher_.publish(point)
            self.get_logger().info(f"군중 감지: {detected_people_count}명, 위치: ({point.x}, {point.y})")

def main(args=None):
    print("haha")
    rclpy.init(args=args)
    node = CCTVCrowdDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
