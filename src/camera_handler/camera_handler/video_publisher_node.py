import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')
        self.publisher_ = self.create_publisher(Image, '/video_stream', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.cap = cv2.VideoCapture(0)
        self.bridge = CvBridge()

        if (not self.cap.isOpened()):
            self.get_logger().error('Error opening the video stream')
            exit(1)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(image_msg)
            self.get_logger().info('Publishing video frame')
        else:
            self.get_logger().error('Error reading frame')

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    video_publisher = VideoPublisher()
    try:
        rclpy.spin(video_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        video_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
