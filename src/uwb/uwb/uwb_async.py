import rclpy
import asyncio
import serial_asyncio
import time
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs
from .SerialAsync import SerialAsync
from .NavBridge import NavBridge


class Uwb(Node):
    def __init__(self):
        super().__init__("uwb_async_node")
        self.get_logger().info("UWB Async Node started.")

        self.pubba = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.bridge = NavBridge()
        self.last_time = time.monotonic()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.loop = asyncio.get_event_loop()
        self.loop.run_until_complete(self.run_serial_connection())

    async def run_serial_connection(self):
        self.get_logger().info("Connecting to serial device...")
        transport, protocol = await serial_asyncio.create_serial_connection(
            asyncio.get_running_loop(),
            lambda: SerialAsync(self.process_message),
            "/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller_CCCJb11A921-if00-port0",
            baudrate=230400,
        )
        self.get_logger().info("Serial connection established.")
        await asyncio.Future()

    def process_message(self, message):
        cur_time = time.monotonic()
        if cur_time - self.last_time >= 2:
            self.get_logger().info(f"{cur_time} vs {self.last_time}")
            goal = self.bridge.convert_message_to_goal(message)
            try:
                transform = self.tf_buffer.lookup_transform(
                    "map",
                    goal.header.frame_id,
                    rclpy.time.Time()
                )
                transformed_goal = tf2_geometry_msgs.do_transform_pose(goal, transform)
                self.get_logger().info(f"Transformed goal: {transformed_goal}")
                self.pubba.publish(transformed_goal)
            except Exception as e:
                self.get_logger().error("Transform failed: " + str(e))
                self.pubba.publish(goal)
            self.last_time = cur_time



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
