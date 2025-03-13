import rclpy
import asyncio
import serial_asyncio
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from .SerialAsync import SerialAsync
from .Message import Message
from .NavBridge import NavBridge
import numpy as np

class Uwb(Node):
  def __init__(self):
      super().__init__("uwb_async_node")
      self.get_logger().info("UWB Async Node started.")

      self.pubba = self.create_publisher(PoseStamped, "/goal_post", 10)
      self.bridge = NavBridge()

      self.loop = asyncio.get_event_loop()
      self.loop.run_until_complete(self.run_serial_connection())

  async def run_serial_connection(self):
      self.get_logger().info("Connecting to serial device...")
      transport, protocol = await serial_asyncio.create_serial_connection(
          asyncio.get_running_loop(),
          lambda: SerialAsync(self.process_message),
            "/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller_CCCJb11A921-if00-port0",
          baudrate=230400
      )
      self.get_logger().info("Serial connection established.")
      await asyncio.Future()

  def process_message(self, message):
      goal = self.bridge.convert_message_to_goal(message)
      self.get_logger().info(f"Goal type: {type(goal)}")
      self.get_logger().info(f"Publishing goal: {goal}")
      self.pubba.publish(goal)

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
