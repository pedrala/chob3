import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from flask import Flask, render_template, Response
import threading

# Flask App Initialization
# app = Flask(__name__)

# Specify the folder where your templates are located
app = Flask(__name__, template_folder='/home/viator/ws/chob3_ws/src/expo_bot/expo_bot/templates')

class SysMonitorNode(Node):
    def __init__(self):
        super().__init__('sys_monitor_node')  # Node name
        self.subscription = self.create_subscription(
            Image,
            '/crowd_image',
            self.image_callback,
            10)
        self.bridge = CvBridge()  # Bridge for converting ROS Image to OpenCV format
        self.latest_frame = None  # Variable to store the latest frame

    def image_callback(self, msg):
        # Convert the ROS2 Image message to an OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.latest_frame = cv_image  # Store the latest frame for Flask

def generate_frames():
    while True:
        if sys_monitor_node.latest_frame is not None:
            try:
                # Convert OpenCV image to JPEG format for Flask
                ret, jpeg = cv2.imencode('.jpg', sys_monitor_node.latest_frame)
                if ret:
                    # Send the JPEG image as a stream
                    frame = jpeg.tobytes()
                    yield (b'--frame\r\n'
                            b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            except Exception as e:
                app.logger.error(f"Error encoding frame: {e}")
        else:
            continue

@app.route('/')
def index():
    print ('setting up webpage')
    return render_template('cam_index.html')

# Flask route to serve the video stream
@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

# @app.route('/video_feed2')
# def video_feed2():
#     return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

# def run_flask():
    
#     app = Flask(__name__, template_folder='./expo_bot/expo_bot/templates')
#     app.run(host='0.0.0.0', port=5000, threaded=True)

def main():
    rclpy.init()
    global sys_monitor_node
    sys_monitor_node = SysMonitorNode()  # ROS2 Node object

    # # Run Flask app in a separate thread
    # flask_thread = threading.Thread(target=run_flask)
    # flask_thread.start()

    app.run(debug=True)
    # ROS2 spin to keep the ROS node running
    rclpy.spin(sys_monitor_node)

    sys_monitor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
