from datetime import datetime
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import sqlite3
import json
import cv2
from flask import Flask, render_template, Response
import threading
import os

# Flask App 설정
app = Flask(__name__, template_folder=os.path.join(os.path.dirname('/home/viator/ws/chob3_ws/src/expo_bot/expo_bot/templates'), 'templates'))

class SysMonitorNode(Node):
    def __init__(self):
        super().__init__('sys_monitor_node')
        
        # 데이터베이스 접근에 대한 스레드 안전을 보장하기 위한 락(lock) 설정
        self.db_lock = threading.Lock()

        # 이미지 토픽 구독
        self.image_subscription = self.create_subscription(
            Image,
            '/crowd_image',
            self.image_callback,
            10)
        
        # 객체 인식된 JSON 데이터 토픽 구독
        self.json_subscription = self.create_subscription(
            String,
            '/detected_object',
            self.json_callback,
            10)
        
        # CvBridge 초기화
        self.bridge = CvBridge()
        self.latest_frame = None  # 최신 이미지 저장
        
        # SQLite3 데이터베이스 설정 및 테이블 생성
        self.conn = sqlite3.connect('detected_objects.db', check_same_thread=False)
        self.cursor = self.conn.cursor()
      
        # Create table if it doesn't exist with all necessary columns
        with self.db_lock:  # Ensure table creation is thread-safe
            self.cursor.execute('''CREATE TABLE IF NOT EXISTS detected_objects
                                (class TEXT, confidence REAL, object_count INTEGER, zone INTEGER, time REAL)''')
        
        # Retrieve current table schema to verify columns
        self.cursor.execute('PRAGMA table_info(detected_objects);')
        columns = [col[1] for col in self.cursor.fetchall()]
        
        # Conditionally add missing columns
        if 'zone' not in columns:
            self.cursor.execute('ALTER TABLE detected_objects ADD COLUMN zone INTEGER')
        if 'time' not in columns:
            self.cursor.execute('ALTER TABLE detected_objects ADD COLUMN time REAL')
        
        self.conn.commit()

    def image_callback(self, msg):
        """ROS2 Image 메시지를 OpenCV 이미지로 변환하여 최신 프레임으로 저장"""
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.latest_frame = cv_image

    def json_callback(self, msg):
        """JSON 데이터 구독 콜백, 데이터를 파싱하여 SQLite에 저장"""
        try:
            data = json.loads(msg.data)  # JSON 문자열 파싱
            for item in data:
                class_name = item.get('class')
                confidence = item.get('confidence')
                object_count = item.get('object_count')
                zone = item.get('zone')
                timestamp = item.get('time')

                # SQLite3에 데이터 저장
                with self.db_lock: ## Use lock and try-except to handle any locked database issues
                    self.cursor.execute('INSERT INTO detected_objects (class, confidence, object_count, zone, time) VALUES (?, ?, ?, ?, ?)',
                                        (class_name, confidence, object_count, zone, timestamp))
                    self.conn.commit()  # 변경 사항 저장
                    self.get_logger().info(f"Saved data to database: {item}")

        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to decode JSON: {e}")
        except Exception as e:
            self.get_logger().error(f"Error saving data: {e}")

    def destroy_node(self):
        """노드 종료 시 데이터베이스 연결 종료"""
        self.conn.close()
        super().destroy_node()


def generate_frames():
    """이미지를 Flask로 스트리밍하는 제너레이터 함수"""
    while True:
        if sys_monitor_node.latest_frame is not None:
            try:
                ret, jpeg = cv2.imencode('.jpg', sys_monitor_node.latest_frame)
                if ret:
                    frame = jpeg.tobytes()
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            except Exception as e:
                app.logger.error(f"Error encoding frame: {e}")
        else:
            continue

@app.route('/')
def index():
    # 데이터베이스에서 객체 데이터 조회
    sys_monitor_node.cursor.execute('SELECT class, confidence, object_count, zone, time FROM detected_objects ORDER BY time DESC LIMIT 10')
    detected_data = sys_monitor_node.cursor.fetchall()

     # 데이터가 정상적으로 조회되었는지 확인
    app.logger.info(f"Fetched data: {detected_data}")

    # 각 엔트리의 타임스탬프를 포맷된 시간 문자열로 변환
    formatted_data = []
    for entry in detected_data:
        class_name, confidence, object_count, zone, timestamp = entry
        formatted_time = datetime.fromtimestamp(timestamp).strftime('%Y-%m-%d %H:%M:%S')
        formatted_data.append((class_name, confidence, object_count, zone, formatted_time))
    

    # 템플릿에 데이터를 전달
    return render_template('cam_index.html', detected_data=formatted_data)

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/get_detected_data')
def get_detected_data():
    # 데이터베이스에서 객체 데이터 조회
    sys_monitor_node.cursor.execute('SELECT class, confidence, object_count, zone, time FROM detected_objects ORDER BY time DESC LIMIT 10')
    detected_data = sys_monitor_node.cursor.fetchall()

    # 각 엔트리의 타임스탬프를 포맷된 시간 문자열로 변환
    formatted_data = []
    for entry in detected_data:
        class_name, confidence, object_count, zone, timestamp = entry
        formatted_time = datetime.fromtimestamp(timestamp).strftime('%Y-%m-%d %H:%M:%S')
        formatted_data.append({
            'class_name': class_name,
            'confidence': confidence,
            'object_count': object_count,
            'zone': zone,
            'time': formatted_time
        })

    return json.dumps(formatted_data)  # JSON 형식으로 반환

def main():
    rclpy.init()
    global sys_monitor_node
    sys_monitor_node = SysMonitorNode()

    flask_thread = threading.Thread(target=lambda: app.run(host='0.0.0.0', port=5000))
    flask_thread.start()

    rclpy.spin(sys_monitor_node)
    sys_monitor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
