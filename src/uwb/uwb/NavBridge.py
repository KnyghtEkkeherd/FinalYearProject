import math
import numpy as np
from geometry_msgs.msg import PoseStamped
from rclpy.clock import Clock
from Filter import Filter


class NavBridge:
    def __init__(self, frame_id="laser_frame", filter_instance=None):
        self.frame_id = frame_id

        self.filter = filter_instance if filter_instance is not None else Filter()

    def convert_message_to_goal(self, message):
        filtered_data = self.filter.process_message(message)
        goal = PoseStamped()
        goal.header.frame_id = self.frame_id
        goal.header.stamp = Clock().now().to_msg()

        goal.pose.position.x = filtered_data["x"]
        goal.pose.position.y = filtered_data["y"]

        goal.pose.position.z = message.elevation / 100.0

        az = filtered_data["azimuth"]
        half_angle = math.radians(az / 2.0)
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = np.sin(half_angle)
        goal.pose.orientation.w = np.cos(half_angle)

        return goal
