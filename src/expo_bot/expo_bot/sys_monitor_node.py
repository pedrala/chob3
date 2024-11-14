import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class SysMonitorNode(Node):
    def __init__(self):
        super().__init__('sys_monitor_node')  # Node name
        self.subscription = self.create_subscription(
            Image,
            '/crowd_image',
            self.image_callback,
            10)
        self.bridge = CvBridge()  # Bridge for converting ROS Image to OpenCV format

    def image_callback(self, msg):
        # Convert the ROS2 Image message to an OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Display the received image
        cv2.imshow("Received Image from /crowd_image", cv_image)
        if cv2.waitKey(1) == ord('q'):
            self.get_logger().info("Shutting down display")
            rclpy.shutdown()

def main():
    rclpy.init()
    node = SysMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
