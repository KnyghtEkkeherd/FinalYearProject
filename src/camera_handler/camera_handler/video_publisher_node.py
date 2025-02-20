import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import subprocess
import time
import os
import time
import distortion_normalizer

class videoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')
        self.publisher_ = self.create_publisher(Image, '/video_stream', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.bridge = CvBridge()

    def timer_callback(self):
        try:
            timestamp = int(time.time())
            temp_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), f'temp/camera_{timestamp}.jpg')
            subprocess.run(['sudo', 'rpicam-still', '--output', temp_file, '--width', '2026', '--height', '1520', '--encoding', 'jpg', '--immediate'])

            # Read the image from the temporary file
            image = np.array(self.read_image(temp_file))
            ros_image = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
            self.publisher_.publish(ros_image)
            self.get_logger().info('Publishing a new image frame')
            os.remove(temp_file)

        except Exception as e:
            self.get_logger().error(f'Failed to capture image: {e}')

    @staticmethod
    def read_image(file_path):
        """Read an image file and convert it to a format suitable for OpenCV."""
        from PIL import Image as PILImage
        image = PILImage.open(file_path)
        return np.array(image)

def main(args=None):
    rclpy.init(args=args)
    video_publisher = videoPublisher()
    rclpy.spin(video_publisher)

    video_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()