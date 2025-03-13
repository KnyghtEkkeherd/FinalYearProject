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

        goal.pose.position.x = float(message.parsed_fields["distance"] * np.cos(message.parsed_fields["azimuth"]))
        goal.pose.position.y = float(message.parsed_fields["distance"] * np.sin(message.parsed_fields["azimuth"]))
        goal.pose.position.z = float(message.parsed_fields["elevation"])

        goal.pose.orientation.x = float(0.0)
        goal.pose.orientation.y = float(0.0)
        goal.pose.orientation.z = float(np.sin(message.parsed_fields["azimuth"] / 2))
        goal.pose.orientation.w = float(np.cos(message.parsed_fields["azimuth"] / 2))

        return goal
