# robot_label.py
# Publishes a TEXT_VIEW_FACING marker above each robot that follows its position
# Visually view robot1 apart from robot2 in RViz.

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration


class RobotLabel(Node):
    def __init__(self):
        super().__init__('robot_label')

        amcl_qos = QoSProfile(depth=10)
        amcl_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        amcl_qos.durability = DurabilityPolicy.VOLATILE

        qos = QoSProfile(depth=10)
        
        #publisher
        self.label_pub = self.create_publisher(MarkerArray, '/game/robot_label', qos)
 
        self.robot1_pose = None
        self.robot2_pose = None

        #subscribe to robot pose
        self.create_subscription(
            PoseWithCovarianceStamped, '/robot1/amcl_pose',
            self.robot1_callback, amcl_qos)
        self.create_subscription(
            PoseWithCovarianceStamped, '/robot2/amcl_pose',
            self.robot2_callback, amcl_qos)

        # Publish at 10 Hz so the label tracks smoothly
        self.create_timer(0.1, self.publish_label)
        self.get_logger().info('Robot Label node started!')

    def robot1_callback(self, msg):
        self.robot1_pose = msg.pose.pose

    def robot2_callback(self, msg):
        self.robot2_pose = msg.pose.pose

    def make_label_marker(self, marker_id, pose, text, r, g, b):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'robot_label'
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        marker.pose.position.x = pose.position.x
        marker.pose.position.y = pose.position.y
        marker.pose.position.z = 0.6  # float above the robot's body
        marker.pose.orientation.w = 1.0

        marker.scale.z = 0.25  # text height in meters

        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 0.9

        marker.text = text
        marker.lifetime = Duration(sec=0, nanosec=0)  # forever, we republish each cycle
        return marker

    def publish_label(self):
        marker_array = MarkerArray()

        if self.robot1_pose is not None:
            marker_array.markers.append(
                self.make_label_marker(101, self.robot1_pose, 'R1', 0.0, 0.0, 1.0))  # blue

        if self.robot2_pose is not None:
            marker_array.markers.append(
                self.make_label_marker(102, self.robot2_pose, 'R2', 1.0, 0.0, 0.0))  # red

        if marker_array.markers:
            self.label_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = RobotLabel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
