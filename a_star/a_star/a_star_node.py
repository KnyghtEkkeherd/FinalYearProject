import numpy as np
from easydict import EasyDict as edict
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from sklearn.cluster import DBSCAN

from utils import *

class AStar(Node):
    def __init__(self):
        super().__init__('a_star')


def main(arg=None):
    rclpy.init(args=arg)
    a_star = AStar()
    rclpy.spin(a_star)
    a_star.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
