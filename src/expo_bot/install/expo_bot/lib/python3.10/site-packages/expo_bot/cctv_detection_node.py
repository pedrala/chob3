import json
import csv
import time
from ultralytics import YOLO
import cv2
import os
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge  # OpenCV 이미지를 ROS2 이미지 메시지로 변환

# CCTVCrowdDetectionNode 클래스 정의
class CCTVCrowdDetectionNode(Node):
    def __init__(self):
        super().__init__('cctv_detection_node')  # 노드 이름 설정
        self.location_publisher_ = self.create_publisher(Point, '/crowd_location', 10)  # 위치 정보 퍼블리셔 생성
        self.image_publisher_ = self.create_publisher(Image, '/crowd_image', 10)  # 이미지 퍼블리셔 생성
        self.bridge = CvBridge()  # CvBridge 인스턴스 생성
        self.blue_box_margin = 50  # 파란색 바운딩 박스의 여백 (픽셀 단위)
        self.model = YOLO('./output/my_best.pt')  # YOLO 모델 로드
        self.cap = cv2.VideoCapture(0)  # 카메라 사용
        self.last_alert = None  # 마지막 알림 시간 추적 (초기화)
        
        # 저장 폴더 확인 및 생성
        if not os.path.exists('./output'):
            os.makedirs('./output')
        
        # CSV 파일을 열어 기록할 준비
        self.csv_file = open('./output/output.csv', mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['filename', 'x1', 'y1', 'x2', 'y2', 'confidence', 'class', 'object_count'])
        
        self.json_data = []  # JSON 파일에 저장할 데이터를 위한 리스트

    def run_detection(self):
        while True:
            success, img = self.cap.read()  # 카메라로부터 이미지 읽기
            results = self.model(img, stream=True)  # YOLO 모델을 이용해 객체 감지

            # 파란색 바운딩 박스의 경계 정의
            height, width, _ = img.shape
            margin = self.blue_box_margin
            bbox_x1, bbox_y1 = margin, margin
            bbox_x2, bbox_y2 = width - margin, height - margin

            # 파란색 바운딩 박스를 화면에 그림
            cv2.rectangle(img, (bbox_x1, bbox_y1), (bbox_x2, bbox_y2), (255, 0, 0), 2)

            object_count = 0  # 물체 카운트 초기화

            for r in results:
                boxes = r.boxes  # 감지된 객체들의 바운딩 박스 정보

                for box in boxes:
                    # 바운딩 박스 좌표 추출
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    center_x = (x1 + x2) // 2  # 객체 중심 x 좌표
                    center_y = (y1 + y2) // 2  # 객체 중심 y 좌표
                    confidence = float(box.conf[0].item())  # 객체의 정확도 (Tensor -> float 변환)
                    class_id = int(box.cls[0].item())  # 객체의 클래스 ID (Tensor -> int 변환)

                    # 클래스 이름을 가져오기 (예: 'car', 'person', etc.)
                    class_name = self.model.names[class_id]

                    # 객체의 중심이 파란색 바운딩 박스 내부에 있는지 확인
                    if bbox_x1 < center_x < bbox_x2 and bbox_y1 < center_y < bbox_y2:
                        # 객체가 바운딩 박스 내부에 들어오면, 색상 반전
                        img[y1:y2, x1:x2] = cv2.bitwise_not(img[y1:y2, x1:x2])

                        # 최근에 알림을 보낸 시간과 비교하여, 일정 시간 간격으로만 알림을 보냄
                        current_time = time.time()
                        if self.last_alert is None or current_time - self.last_alert >= 1:  # 1초 간격
                            # 침입 알림: 객체의 중심 위치를 퍼블리시
                            intrusion_location = Point(x=float(center_x), y=float(center_y), z=0.0)
                            self.location_publisher_.publish(intrusion_location)
                            self.get_logger().info(f"물체가 바운딩 박스 안으로 들어옴: 위치 ({center_x}, {center_y})")
                            self.last_alert = current_time  # 알림 보낸 시간 업데이트

                        # 객체 카운트 증가
                        object_count += 1

                    # 객체의 바운딩 박스를 화면에 그림
                    cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    # 클래스 이름과 confidence를 표시
                    label = f"{class_name} {confidence:.2f}"
                    cv2.putText(img, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            # 물체가 2개 이상인 경우에만 이미지 저장
            if object_count >= 2:
                filename = f'./output/{time.time()}.jpg'
                cv2.imwrite(filename, img)

                # CSV 파일에 객체 정보 저장
                self.csv_writer.writerow([filename, x1, y1, x2, y2, confidence, class_name, object_count])

                # JSON 파일에 객체 정보 저장
                self.json_data.append({
                    'filename': filename,
                    'x1': x1,
                    'y1': y1,
                    'x2': x2,
                    'y2': y2,
                    'confidence': confidence,
                    'class': class_name,
                    'object_count': object_count
                })

            # 이미지를 ROS2 이미지 메시지로 변환하여 퍼블리시
            image_message = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
            self.image_publisher_.publish(image_message)

            # 화면에 결과 표시
            cv2.imshow('crowd_location', img)

            if cv2.waitKey(1) == ord('q'):
                break

        # 카메라 자원 해제 및 창 닫기
        self.cap.release()
        cv2.destroyAllWindows()

        # JSON 파일로 저장
        with open('./output/output.json', 'w') as json_file:
            json.dump(self.json_data, json_file, indent=4)

        # CSV 파일 닫기
        self.csv_file.close()

# ROS2 노드 실행 함수
def main():
    rclpy.init()
    node = CCTVCrowdDetectionNode()  # CCTVCrowdDetectionNode 객체 생성
    node.run_detection()  # 객체 탐지 실행
    node.destroy_node()  # 노드 종료
    rclpy.shutdown()  # ROS2 종료

if __name__ == "__main__":
    main()
