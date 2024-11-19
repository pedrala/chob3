import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from ultralytics import YOLO
from sensor_msgs.msg import Image  # 이미지를 퍼블리시하기 위해 필요한 메시지
from cv_bridge import CvBridge  # OpenCV 이미지를 ROS2 이미지 메시지로 변환
import cv2  # OpenCV
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy

class AMRControlNode(Node):
    def __init__(self):
        super().__init__('amr_control_node')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # BEST_EFFORT 또는 RELIABLE
            depth=10
        )
        
        # 현재 위치를 발행할 토픽 생성
        self.position_publisher_ = self.create_publisher(Point, '/amr_current_position', qos_profile)
        
        # 카메라 영상을 발행할 토픽 생성
        self.image_publisher_ = self.create_publisher(CompressedImage, '/amr_camera_image', qos_profile)
        self.bridge = CvBridge()  # OpenCV 브리지 생성
        
        # 카메라 초기화
        for device_index in range(2):  # 최대 2개의 카메라 확인
            self.cap = cv2.VideoCapture(device_index)
            if self.cap.isOpened():
                self.get_logger().info(f"카메라 초기화 성공: /dev/video{device_index}")
                break
        else:
            self.get_logger().error("카메라 초기화 실패! 연결된 카메라를 확인하세요.")
            return

        # YOLO 모델 로드
        model_path = '/home/rokey9/chob3/chob3_ws/src/expo_bot/resource/robot.pt'
        if not os.path.exists(model_path):
            self.get_logger().error(f"YOLO 모델 파일이 없습니다: {model_path}")
        else:
            self.model = YOLO(model_path)
            #self.model = YOLO('./src/expo_bot/resource/robot.pt')  # YOLO 모델 로드
     

        # Odometry 또는 AMCL의 위치 정보를 구독
        self.pose_subscription_ = self.create_subscription(
            Odometry,
            '/odom',  # 또는 '/amcl_pose'로 변경 가능
            self.pose_callback,
            10
        )
        # 일정 주기로 위치와 영상을 퍼블리시 (초 단위로 설정 가능)
        self.timer_ = self.create_timer(1.0, self.publish_data)
        # 로봇의 현재 위치
        self.current_position = Point()

    def run_detection(self):
        while True:
            # 카메라에서 프레임 읽기
            ret, img = self.cap.read()
            if not ret or img is None:
                self.get_logger().error("카메라 프레임을 읽을 수 없습니다. 연결 상태를 확인하세요.")
                return  # 비정상적인 경우 처리 중단

            # YOLO 모델 적용
            try:
                results = self.model(img, stream=True)
                # (이하 YOLO 객체 인식 처리 코드)
            except Exception as e:
                self.get_logger().error(f"YOLO 처리 중 오류 발생: {e}")
                return

            for r in results:
                boxes = r.boxes  # 감지된 객체들의 바운딩 박스 정보

                for box in boxes:
                    # 바운딩 박스 좌표 추출
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    confidence = float(box.conf[0].item())  # 객체의 정확도 (Tensor -> float 변환)
                    class_id = int(box.cls[0].item())  # 객체의 클래스 ID (Tensor -> int 변환)

                    # 클래스 이름을 가져오기 (예: 'car', 'person', etc.)
                    class_name = self.model.names[class_id]

                    # 객체의 바운딩 박스를 화면에 그림
                    cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    # 클래스 이름과 confidence를 표시
                    label = f"{class_name} {confidence:.2f}"
                    cv2.putText(img, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            # 이미지를 ROS2 이미지 메시지로 변환하여 퍼블리시
            # 이미지 압축 후 퍼블리시
            _, encoded_image = cv2.imencode('.jpg', img)  # 이미지 JPEG 형식으로 압축
            image_message = CompressedImage()
            image_message.header.stamp = self.get_clock().now().to_msg()  # to_msg()로 Time 객체 변환
            image_message.format = "jpeg"
            image_message.data = np.array(encoded_image).tobytes()  # 압축된 이미지를 바이트로 변환

            #image_message = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
            self.image_publisher_.publish(image_message)

            # 화면에 결과 표시
            cv2.imshow('amr_camera_image', img)

            if cv2.waitKey(1) == ord('q'):
                break
            
        # 카메라 자원 해제 및 창 닫기
        self.cap.release()
        cv2.destroyAllWindows()

    # Odometry로부터 로봇 위치 정보 받기
    def pose_callback(self, msg):
        # Odometry 메시지에서 위치 정보 가져오기image_messagex
        self.current_position.y = msg.pose.pose.position.y
        self.current_position.z = 0.0  # 2D 위치
        self.get_logger().info(f"로봇 위치 업데이트: x={self.current_position.x}, y={self.current_position.y}")
    
    # 현재 좌표와 영상을 퍼블리시하는 함수
    def publish_data(self):
        # 좌표를 퍼블리시
        self.position_publisher_.publish(self.current_position)
        self.get_logger().info(f"AMR 현재 위치 발행: x={self.current_position.x}, y={self.current_position.y}")

    def destroy_node(self):
        # 카메라 자원 해제
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

# ROS2 노드 실행 함수
def main():
    rclpy.init()
    node = AMRControlNode()
    node.run_detection()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
