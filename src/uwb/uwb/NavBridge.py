import math
import numpy as np
from geometry_msgs.msg import PoseStamped
from .Message import Message
from rclpy.clock import Clock

class NavBridge:
    def __init__(self, frame_id="laser_frame"):
        self.frame_id = frame_id

    def convert_message_to_goal(self, message: Message):
        goal = PoseStamped()
        goal.header.frame_id = self.frame_id
        goal.header.stamp = Clock().now().to_msg()

        goal.pose.position.x = message.distance * np.cos(math.radians(message.azimuth)) / 100.0
        goal.pose.position.y = message.distance * np.sin(math.radians(message.azimuth)) / 100.0
        goal.pose.position.z = message.elevation / 100.0

        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = np.sin(math.radians(message.azimuth / 2.0))
        goal.pose.orientation.w = np.cos(math.radians(message.azimuth / 2.0))

        return goal
