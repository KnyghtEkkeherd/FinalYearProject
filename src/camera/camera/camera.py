import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_srvs.srv import Trigger
import os


class CameraService(Node):
    def __init__(self):
        super().__init__("camera_service")
        self.publisher_ = self.create_publisher(Image, "camera/image", 10)
        self.bridge = CvBridge()
        self.srv = self.create_service(Trigger, "take_photo", self.take_photo_callback)

    def take_photo_callback(self, request, response):
        # Use rpicam-still to capture the image
        os.system("rpicam-still -o test1.jpg --vflip --hflip")

        # Read the image and convert to ROS Image message
        frame = self.bridge.imgmsg_to_cv2("test1.jpg", encoding="bgr8")
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.publisher_.publish(msg)

        response.success = True
        response.message = "Photo captured and published"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = CameraService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
