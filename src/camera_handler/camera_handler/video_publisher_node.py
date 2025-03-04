from cv2 import face
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import subprocess
import os
import .distortion_normalizer
import .recognition_utils
import time
from camera_handler.srv import TriggerImage, TriggerImageRecognition

class CameraHandler(Node):
    def __init__(self):
        super().__init__('facial_recognition')
        from std_srvs.srv import Trigger
        self.get_image_srv = self.create_service(TriggerImage, 'get_image', self.camera_callback)
        self.facial_recognition_srv = self.create_service(TriggerImageRecognition, 'facial_recognition', self.facial_recognition_callback)

    def facial_recognition_callback(self, request, response):
        try:
            timestamp = int(time.time())
            temp_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), f'temp/camera_{timestamp}.jpg')
            subprocess.run(['sudo', 'rpicam-still', '--output', temp_file, '--width', '2026', '--height', '1520', '--encoding', 'jpg', '--immediate'])

            restored_image = f"temp/camera_{timestamp}_restored.jpg"
            distortion_normalizer.image_restore(temp_file, restored_image)
            bridge = CvBridge()
            image_msg = bridge.cv2_to_imgmsg(cv2.imread(restored_image), encoding="bgr8")
            face_locations, face_names = recognition_utils.recognize_faces(restored_image)
            response.image = image_msg
            # TODO: publish the face locations and names

            os.remove(temp_file)
            os.remove(restored_image)
            self.get_logger().info('Facial recognition completed')

        except Exception as e:
            self.get_logger().error(f'Failed to capture image: {e}')

    def camera_callback(self, request, response):
        try:
            timestamp = int(time.time())
            temp_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), f'temp/camera_{timestamp}.jpg')
            subprocess.run(['sudo', 'rpicam-still', '--output', temp_file, '--width', '2026', '--height', '1520', '--encoding', 'jpg', '--immediate'])

            restored_image = f"temp/camera_{timestamp}_restored.jpg"
            distortion_normalizer.image_restore(temp_file, restored_image)
            bridge = CvBridge()
            image_msg = bridge.cv2_to_imgmsg(cv2.imread(restored_image), encoding="bgr8")
            response.image = image_msg

            os.remove(temp_file)
            os.remove(restored_image)
            self.get_logger().info('Image capture completed')

        except Exception as e:
            self.get_logger().error(f'Failed to capture image: {e}')

    @staticmethod
    def read_image(file_path):
        from PIL import Image as PILImage
        image = PILImage.open(file_path)
        return np.array(image)

def main(args=None):
    rclpy.init(args=args)
    camera_handler = CameraHandler()
    rclpy.spin(camera_handler)

    camera_handler.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
