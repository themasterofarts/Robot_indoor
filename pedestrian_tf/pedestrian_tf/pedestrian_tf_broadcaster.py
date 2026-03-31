import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster


class PedestrianTFBroadcaster(Node):
    def __init__(self):
        super().__init__('pedestrian_tf_broadcaster')

        self.declare_parameter('input_topic', '/actor/pose')
        self.declare_parameter('parent_frame', 'map')
        self.declare_parameter('child_frame', 'pedestrian')

        self.input_topic = self.get_parameter('input_topic').value
        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame = self.get_parameter('child_frame').value

        self.tf_broadcaster = TransformBroadcaster(self)

        self.subscription = self.create_subscription(
            PoseStamped,
            self.input_topic,
            self.pose_callback,
            10
        )

        self.get_logger().info(
            f"Subscribed to {self.input_topic}, publishing TF "
            f"{self.parent_frame} -> {self.child_frame}"
        )

    def pose_callback(self, msg: PoseStamped) -> None:
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()

        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame

        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z

        t.transform.rotation = msg.pose.orientation

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = PedestrianTFBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()