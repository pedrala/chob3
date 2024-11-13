import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        self.publisher_ = self.create_publisher(Bool, '/people_detection', 10)
        self.timer = self.create_timer(1.0, self.detect_people)  # 1초마다 사람 감지

    def detect_people(self):
        # 샘플 데이터로 사람 감지 (실제 코드에서는 컴퓨터 비전 모델 사용)
        people_detected = True  # 샘플 값
        
        # 감지 결과 발행
        msg = Bool(data=people_detected)
        self.publisher_.publish(msg)
        self.get_logger().info(f"사람 감지 상태: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
