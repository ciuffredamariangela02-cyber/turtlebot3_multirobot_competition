# goal_function.py
# This node implements the goal selection strategy for a single robot.
# It subscribes to:
# - Active goals: /game/goals
# - Own position: /robotX/amcl_pose
# - Opponent position: /robotY/amcl_pose
# - Game score: /game/score
# It publishes:
# - Selected goal to Nav2: /robotX/goal_pose

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, PoseStamped
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
import math
import json

# Strategy parameters
ALPHA = 1.0   # weight for own distance (higher = prefer closer goals)
BETA = 0.5    # weight for competitive advantage (higher = prefer blocking opponent)
GOAL_REACHED_THRESHOLD = 0.3  # meters


class GoalFunction(Node):

    def __init__(self):
        super().__init__('goal_function')

        # Robot name parameter (robot1 or robot2)
        self.declare_parameter('robot_name', 'robot1')
        self.robot_name = self.get_parameter('robot_name').value

        # Opponent name
        self.opponent_name = 'robot2' if self.robot_name == 'robot1' else 'robot1'

        # Use namespace if robot_name is set
        self.use_namespace = self.declare_parameter('use_namespace', False).value

        qos = QoSProfile(depth=10)

        # Own pose topic - with or without namespace
        own_pose_topic = f'/{self.robot_name}/amcl_pose' if self.use_namespace else '/amcl_pose'
        opponent_pose_topic = f'/{self.opponent_name}/amcl_pose'

        # Nav2 action topic - with or without namespace
        nav2_action = f'/{self.robot_name}/navigate_to_pose' if self.use_namespace else '/navigate_to_pose'

        # Subscribers
        self.create_subscription(
            PoseArray,
            '/game/goals',
            self.goals_callback,
            qos)

        pose_qos = QoSProfile(depth=10)
        pose_qos.reliability = ReliabilityPolicy.RELIABLE
        pose_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.create_subscription(
            PoseWithCovarianceStamped,
            own_pose_topic,
            self.own_pose_callback,
            pose_qos)

        # self.create_subscription(
        #     PoseWithCovarianceStamped,
        #     opponent_pose_topic,
        #     self.opponent_pose_callback,
        #     qos)

        self.create_subscription(
            String,
            '/game/score',
            self.score_callback,
            qos)

        # Nav2 Action Client
        self.nav2_client = ActionClient(
            self,
            NavigateToPose,
            nav2_action)

        # State
        self.goals = []
        self.own_pose = None
        self.opponent_pose = None
        self.current_goal = None
        self.game_over = False
        self.navigating = False

        # Timer for goal selection
        self.create_timer(1.0, self.select_and_send_goal)

        self.get_logger().info(f'{self.robot_name} Goal Function started!')
        self.get_logger().info(f'Own pose topic: {own_pose_topic}')
        self.get_logger().info(f'Nav2 action: {nav2_action}')

    def goals_callback(self, msg):
        """Receive active goals from Game Master."""
        self.goals = [{'x': p.position.x, 'y': p.position.y} for p in msg.poses]

    def own_pose_callback(self, msg):
        self.own_pose = msg.pose.pose

    def opponent_pose_callback(self, msg):
        self.opponent_pose = msg.pose.pose

    def score_callback(self, msg):
        data = json.loads(msg.data)
        self.game_over = data.get('game_over', False)

    def euclidean_distance(self, pose, goal):
        """Calculate Euclidean distance between a pose and a goal."""
        dx = pose.position.x - goal['x']
        dy = pose.position.y - goal['y']
        return math.sqrt(dx**2 + dy**2)

    def score_goal(self, goal):
        """Calculate the score for a goal using the greedy strategy.

        score = -alpha * own_distance + beta * (opponent_distance - own_distance)

        Higher score = better goal to pursue.
        - own_distance: how far I am from the goal
        - opponent_distance: how far the opponent is from the goal
        - alpha: weight for my distance
        - beta: weight for competitive advantage
        """
        if self.own_pose is None:
            return 0.0

        own_dist = self.euclidean_distance(self.own_pose, goal)

        if self.opponent_pose is not None:
            opp_dist = self.euclidean_distance(self.opponent_pose, goal)
            competitive_advantage = opp_dist - own_dist
        else:
            competitive_advantage = 0.0  # no info about opponent

        return -ALPHA * own_dist + BETA * competitive_advantage

    def select_best_goal(self):
        """Select the best goal using the greedy scoring function."""
        if not self.goals or self.own_pose is None:
            return None

        best_goal = max(self.goals, key=lambda g: self.score_goal(g))
        return best_goal

    def send_goal_to_nav2(self, goal):
        """Send the selected goal to Nav2."""
        if not self.nav2_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Nav2 action server not available')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = goal['x']
        goal_msg.pose.pose.position.y = goal['y']
        goal_msg.pose.pose.orientation.w = 1.0

        self.nav2_client.send_goal_async(goal_msg)
        self.current_goal = goal
        self.navigating = True
        self.get_logger().info(
            f'{self.robot_name} navigating to goal ({goal["x"]:.2f}, {goal["y"]:.2f})')

    def is_goal_reached(self):
        """Check if the current goal has been reached."""
        if self.current_goal is None or self.own_pose is None:
            return False
        return self.euclidean_distance(self.own_pose, self.current_goal) < GOAL_REACHED_THRESHOLD

    def select_and_send_goal(self):
        self.get_logger().info(f'Goals: {len(self.goals)}, Own pose: {self.own_pose is not None}')
        """Main loop: select best goal and send to Nav2."""
        if self.game_over:
            return

        if not self.goals or self.own_pose is None:
            return

        # If current goal reached or no goal set, select a new one
        if self.current_goal is None or self.is_goal_reached():
            best_goal = self.select_best_goal()
            if best_goal:
                self.send_goal_to_nav2(best_goal)


def main(args=None):
    rclpy.init(args=args)
    node = GoalFunction()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()