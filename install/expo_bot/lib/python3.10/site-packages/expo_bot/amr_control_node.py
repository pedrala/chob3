import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, Float32

class AMRControlNode(Node):
    def __init__(self):
        super().__init__('amr_control_node')
        self.create_subscription(Point, '/crowd_location', self.set_target_location, 10)
        self.move_to_target = self.create_publisher(Float32, '/move_to_target', 10)
        
        # 목표 위치 변수 초기화
        self.target_location = None
        self.current_location = Point(x=0.0, y=0.0, z=0.0)

    def set_target_location(self, msg):

        print('msg:'+msg)
        # AMR이 목적지를 받으면 목표 위치를 설정하고 이동 시작
        self.target_location = msg
        self.get_logger().info(f"목표 위치 설정: ({msg.x}, {msg.y})로 이동")
        self.move_to_target.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = AMRControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
