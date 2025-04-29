import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs

class PoseTransformer(Node):
    def __init__(self):
        super().__init__('pose_transformer')
        self.get_logger().info("UWB pose transformer initialized.")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.goal_pose_subscriber = self.create_subscription(
            PoseStamped,
            '/goal_pose_uwb',
            self.listener_callback,
            10)

        self.transformed_pose_publisher = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10)

    def listener_callback(self, msg):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'uwb_frame',
                rclpy.time.Time())
            transformed_pose = tf2_geometry_msgs.do_transform_pose(msg, transform)
            self.get_logger().info(f'Transformed Pose: {transformed_pose}')

            self.transformed_pose_publisher.publish(transformed_pose)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f'Transform error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = PoseTransformer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
