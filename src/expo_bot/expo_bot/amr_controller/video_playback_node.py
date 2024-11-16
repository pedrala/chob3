import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class VideoPlaybackNode(Node):
    def __init__(self):
        super().__init__('video_playback_node')
        self.create_subscription(Bool, '/play_promo_video', self.play_video_callback, 10)

    def play_video_callback(self, msg):
        # 재생 요청이 들어오면 영상 재생 (True: 재생, False: 중지)
        if msg.data:
            self.get_logger().info("홍보 영상 재생 중...")
        else:
            self.get_logger().info("홍보 영상 재생 중지")

def main(args=None):
    rclpy.init(args=args)
    node = VideoPlaybackNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
