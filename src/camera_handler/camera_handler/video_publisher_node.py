import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from picamera import PiCamera
from io import BytesIO
from time import sleep
import cv2
import numpy as np

# https://picamera.readthedocs.io/en/release-1.13/install.html
# sudo apt-get update
# sudo apt-get install python-picamera python3-picamera
#

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.publisher_ = self.create_publisher(Image, '/video_stream', 10)
        self.camera = PiCamera()
        self.camera.resolution = (2028, 1520)
        self.camera.start_preview()
        sleep(2)  # Camera warm-up time

        self.capture_and_publish()

    def capture_and_publish(self):
        my_stream = BytesIO()
        self.camera.capture(my_stream, 'jpeg')
        my_stream.seek(0)

        # Convert the captured image to a ROS message
        img_msg = Image()
        img_msg.width = 2028
        img_msg.height = 1520
        img_msg.encoding = "jpeg"
        img_msg.data = my_stream.getvalue()
        img_msg.step = img_msg.width * 3  # Assuming 3 channels (RGB)

        self.publisher_.publish(img_msg)
        self.get_logger().info('Published image.')

def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    rclpy.spin(camera_node)
    camera_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
