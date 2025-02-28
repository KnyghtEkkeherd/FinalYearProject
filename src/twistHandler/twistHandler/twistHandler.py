import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class twistHandler (Node):

    def __init__(self):
        super().__init__('twistHandler')
        self.publisher = self.create_publisher(TwistStamped, '/diff_cont/cmd_vel', 10)
        self.subscription = self.create_subscription(Twist, '/cmd_vel_smoothed', self.listener_callback, 10)

    def listener_callback(self, msg):
        twist_stamped_msg = TwistStamped()
        twist_stamped_msg.header.stamp = self.get_clock().now().to_msg()
        twist_stamped_msg.twist = msg
        self.publisher.publish(twist_stamped_msg)

def main(args=None):
    rclpy.init(args=args)
    minimal_pubsub_node = twistHandler()
    rclpy.spin(minimal_pubsub_node)
    minimal_pubsub_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
