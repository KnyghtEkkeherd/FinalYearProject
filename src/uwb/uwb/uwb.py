import rclpy
import time
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs
from .SerialMT import SerialMT
from .NavBridge import NavBridge

class Uwb(Node):
    def __init__(self):
        super().__init__("uwb_node")
        self.get_logger().info("UWB Node started.")
        self.pubba = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.bridge = NavBridge()
        self.last_time = time.monotonic()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.serial = SerialMT(port="/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller_CCCJb11A921-if00-port0",
          message_callback=self.process_message)
        self.get_logger().info("Serial connection established.")

        self.serial.send_command(b"\x20\x01")

    def process_message(self, message):
        # cur_time = time.monotonic()
        # if cur_time - self.last_time >= 0:
            # self.get_logger().info(f"{cur_time} vs {self.last_time}")
        goal = self.bridge.convert_message_to_goal(message)
        goal.header.stamp = self.get_clock().now().to_msg()
        try:
            transform = self.tf_buffer.lookup_transform(
                "map",
                goal.header.frame_id,
                goal.header.stamp,
                timeout=Duration(seconds=1.0)
            )
            self.get_logger().info(f"Transform: {transform}")
            self.get_logger().info(f"Goal: {goal}")
            tf2_geometry_msgs.do_transform_pose(goal, transform)
            # self.get_logger().info(f"Transformed goal: {transformed_goal}")
            goal.pose.position.x = transform.transform.translation.x
            goal.pose.position.y = transform.transform.translation.y
            self.pubba.publish(goal)
        except Exception as e:
            self.get_logger().error("Transform failed: " + str(e))
            self.pubba.publish(goal)
            # self.last_time = cur_time

    def destroy_node(self):
        self.serial.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = Uwb()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
