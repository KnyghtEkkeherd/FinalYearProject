from cv2 import face
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import subprocess
import os
import distortion_normalizer
import recognition_utils
import time

class facialRecognition(Node):
    def __init__(self):
        super().__init__('facial_recognition')
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        try:
            timestamp = int(time.time())
            temp_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), f'temp/camera_{timestamp}.jpg')
            subprocess.run(['sudo', 'rpicam-still', '--output', temp_file, '--width', '2026', '--height', '1520', '--encoding', 'jpg', '--immediate'])

            restored_image = f"temp/camera_{timestamp}_restored.jpg"
            distortion_normalizer.image_restore(temp_file, restored_image)
            face_locations, face_names = recognition_utils.recognize_faces(restored_image)
            self.get_logger().info('Facial recognition completed')

            os.remove(temp_file)
            os.remove(restored_image)

        except Exception as e:
            self.get_logger().error(f'Failed to capture image: {e}')

    @staticmethod
    def read_image(file_path):
        from PIL import Image as PILImage
        image = PILImage.open(file_path)
        return np.array(image)

def main(args=None):
    rclpy.init(args=args)
    video_publisher = facialRecognition()
    rclpy.spin(video_publisher)

    video_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
