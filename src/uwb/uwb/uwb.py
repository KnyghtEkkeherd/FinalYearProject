import rclpy
import asyncio
import serial_asyncio
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from . import SerialAsync, Message
from . import NavBridge

class Uwb(Node):
  def __init__(self):
      super().__init__("uwb_async_node")
      self.get_logger().info("UWB Async Node started.")

      self.pubba = self.create_publisher(PoseStamped, "goal", 10)
      self.bridge = NavBridge()

      self.loop = asyncio.get_event_loop()
      self.loop.run_until_complete(self.run_serial_connection())

  async def run_serial_connection(self):
      self.get_logger().info("Connecting to serial device...")
      transport, protocol = await serial_asyncio.create_serial_connection(
          asyncio.get_running_loop(),
          lambda: SerialAsync(self.process_message),
          "/dev/tty.PL2303G-USBtoUART1130",
          baudrate=230400
      )
      self.get_logger().info("Serial connection established.")
      await asyncio.Future()

  def process_message(self, msg):
      goal = self.bridge.convert_message_to_goal(msg)
      self.pubba.publish(goal)
      self.get_logger().info(f"Publishing goal: {goal}")

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