# tf_relay.py


import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan


class TFRelay(Node):

    def __init__(self):
        super().__init__('tf_relay')

        self.declare_parameter('robot_namespaces', ['robot1', 'robot2'])
        self.robot_namespaces = self.get_parameter('robot_namespaces').value

        tf_qos = QoSProfile(depth=100)
        tf_qos.reliability = ReliabilityPolicy.RELIABLE
        tf_qos.durability = DurabilityPolicy.VOLATILE

        scan_qos = QoSProfile(depth=10)
        scan_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        scan_qos.durability = DurabilityPolicy.VOLATILE

        # Publisher on global /tf
        self.tf_pub = self.create_publisher(TFMessage, '/tf', tf_qos)

        for ns in self.robot_namespaces:
            # Subscribe to each robot's namespaced /tf topic
            self.create_subscription(
                TFMessage,
                f'/{ns}/tf',
                lambda msg, namespace=ns: self.tf_callback(msg, namespace),
                tf_qos)

            # Subscribe to each robot's scan and republish with correct frame_id
            scan_pub = self.create_publisher(LaserScan, f'/{ns}/scan_relay', scan_qos)
            self.create_subscription(
                LaserScan,
                f'/{ns}/scan',
                lambda msg, namespace=ns, pub=scan_pub: self.scan_callback(msg, namespace, pub),
                scan_qos)

            self.get_logger().info(f'Subscribed to /{ns}/tf and /{ns}/scan')

        self.get_logger().info('TF and Scan Relay started!')

    def tf_callback(self, msg, namespace):
        """Republish TF frames as-is on global /tf without modifying frame IDs."""
        self.tf_pub.publish(msg)

    def scan_callback(self, msg, namespace, pub):
        """Republish scan with namespace-prefixed frame_id."""
        new_msg = LaserScan()
        new_msg.header = msg.header
        new_msg.header.frame_id = f'{namespace}/{msg.header.frame_id}'
        new_msg.angle_min = msg.angle_min
        new_msg.angle_max = msg.angle_max
        new_msg.angle_increment = msg.angle_increment
        new_msg.time_increment = msg.time_increment
        new_msg.scan_time = msg.scan_time
        new_msg.range_min = msg.range_min
        new_msg.range_max = msg.range_max
        new_msg.ranges = msg.ranges
        new_msg.intensities = msg.intensities
        pub.publish(new_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TFRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()