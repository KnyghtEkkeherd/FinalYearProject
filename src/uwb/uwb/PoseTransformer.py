import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs
import PyKDL

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
                msg.header.stamp
                )
            print(type(msg))
            print(type(transform))
            transformed_pose = self.do_transform_pose(msg, transform)
            self.get_logger().info(f'Transformed Pose: {transformed_pose}')

            self.transformed_pose_publisher.publish(transformed_pose)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f'Transform error: {e}')

    def transform_to_kdl(self, t):
        return PyKDL.Frame(
                PyKDL.Rotation.Quaternion(t.transform.rotation.x, t.transform.rotation.y,
                    t.transform.rotation.z, t.transform.rotation.w),
                PyKDL.Vector(t.transform.translation.x, 
                    t.transform.translation.y, 
                 t.transform.translation.z)
                 )

    def do_transform_pose(self, pose, transform):
        f = self.transform_to_kdl(transform) * PyKDL.Frame(PyKDL.Rotation.Quaternion(pose.pose.orientation.x, pose.pose.orientation.y,
            pose.pose.orientation.z, pose.pose.orientation.w), PyKDL.Vector(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z))
        res = PoseStamped()
        res.pose.position.x = f.p[0]
        res.pose.position.y = f.p[1]
        res.pose.position.z = f.p[2]
        (res.pose.orientation.x, res.pose.orientation.y, res.pose.orientation.z, res.pose.orientation.w) = f.M.GetQuaternion()
        res.header = transform.header
        return res

def main(args=None):
    rclpy.init(args=args)
    node = PoseTransformer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
