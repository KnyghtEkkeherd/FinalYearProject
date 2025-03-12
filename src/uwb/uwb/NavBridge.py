import numpy as np
from geometry_msgs.msg import PoseStamped
from rclpy import time
from . import Message

class NavBridge:
    def __init__(self, frame_id="map"):
        self.frame_id = frame_id

    def convert_message_to_goal(self, message: Message):
        goal = PoseStamped()
        goal.header.frame_id = self.frame_id
        goal.header.stamp = time.Time()

        goal.pose.position.x = message.parsed_fields["distance"] * np.cos(message.parsed_fields["azimuth"])
        goal.pose.position.y = message.parsed_fields["distance"] * np.sin(message.parsed_fields["azimuth"])
        goal.pose.position.z = message.parsed_fields["elevation"]

        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = np.sin(message.parsed_fields["azimuth"] / 2)
        goal.pose.orientation.w = np.cos(message.parsed_fields["azimuth"] / 2)

        return goal