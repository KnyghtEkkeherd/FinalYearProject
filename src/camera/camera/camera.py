import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_srvs.srv import Trigger
import os
import random
import cv2


class CameraService(Node):
    def __init__(self, no_webcam=False):
        super().__init__("camera_service")
        self.publisher_ = self.create_publisher(Image, "camera/image", 10)
        self.bridge = CvBridge()
        self.srv = self.create_service(Trigger, "take_photo", self.take_photo_callback)
        self.no_webcam = no_webcam

        self.random_images = [
            os.path.join("/fyp_ws/src/camera/camera/images", "img1.jpg"),
            os.path.join("/fyp_ws/src/camera/camera/images", "img2.jpg"),
            os.path.join("/fyp_ws/src/camera/camera/images", "img3.jpg"),
        ]
        self.target_size = (640, 480)

    def take_photo_callback(self, request, response):
        print("Taking photo...")
        if self.no_webcam:
            # Select a random image
            random_img_path = random.choice(self.random_images)
            frame = cv2.imread(random_img_path)
        else:
            # Use rpicam-still to capture the image
            os.system("rpicam-still -o test1.jpg --vflip --hflip")
            frame = cv2.imread("test1.jpg")

        if frame is None:
            response.success = False
            response.message = f"Failed to capture image from {random_img_path}"
            return response

        # Crop and resize the image
        frame = self.crop_and_resize(frame, self.target_size)

        # Convert to ROS Image message
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.publisher_.publish(msg)

        response.success = True
        response.message = "Photo captured and published"
        return response

    def crop_and_resize(self, image, target_size):
        h, w = image.shape[:2]
        target_w, target_h = target_size
        center_x, center_y = w // 2, h // 2

        # Calculate cropping box
        crop_x1 = max(0, center_x - target_w // 2)
        crop_y1 = max(0, center_y - target_h // 2)
        crop_x2 = min(w, center_x + target_w // 2)
        crop_y2 = min(h, center_y + target_h // 2)

        # Crop and resize the image
        cropped = image[crop_y1:crop_y2, crop_x1:crop_x2]
        resized = cv2.resize(cropped, target_size, interpolation=cv2.INTER_AREA)

        return resized


def main(args=None):
    rclpy.init(args=args)
    node = CameraService(no_webcam=False)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
