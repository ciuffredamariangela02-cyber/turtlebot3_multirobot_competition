# game_master.py
# This node is the arbiter of the game. Its main rule consists in:
# - Generate a series of goals in the environment
# - Publish the position of the active goals (a goal become inactive if a robot reaches it)
# - Monitor the position of the robots
# - Gives points to the robots that reach the goals
# - Publish the score of each robot continuosly
# - Declare the winner at the end of the game

# It publishes:
# - The position of the active goals: /game/goals
# - The score of each robot: /game/score

# It subscribes to:
# - The position of the all robots: /robot/pose

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose
from std_msgs.msg import String
import random
import math
import json
from nav_msgs.msg import OccupancyGrid
 
# Game parameters
NUM_GOALS = 10
GOAL_RADIUS = 0.3
ARENA_X_MIN = -2.0
ARENA_X_MAX = 2.0
ARENA_Y_MIN = -2.0
ARENA_Y_MAX = 2.0
PUBLISH_RATE = 1.0  # Hz
MIN_GOAL_DISTANCE = GOAL_RADIUS * 2  # minimum distance between goals

class GameMaster(Node):
 
    def __init__(self):
        super().__init__('game_master')
 
        qos = QoSProfile(depth=10)

        map_qos = QoSProfile(depth=1)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
 
        # Publisher
        self.goals_pub = self.create_publisher(PoseArray, '/game/goals', qos)
        self.score_pub = self.create_publisher(String, '/game/score', qos)
 
        # Subscriber robot position
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.robot1_pose_callback,
            qos)
 
        #self.create_subscription(
        #    PoseWithCovarianceStamped,
        #    '/robot2/amcl_pose',
         #   self.robot2_pose_callback,
        #    qos)

        # Subscriber Nav2 map
        self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            map_qos)
        
        # Play state
        self.robot1_pose = None
        self.robot2_pose = None
        self.score = {'robot1': 0, 'robot2': 0}
        self.goals = []

        self.game_over = False
        self.map = None  # Nav2 OccupancyGrid map
 
        # Pubblication timer
        self.create_timer(1.0 / PUBLISH_RATE, self.game_loop)
 
        self.get_logger().info('Game Master started!')
        self.get_logger().info(f'Goals generated: {len(self.goals)}')

    def map_callback(self, msg):
        """Receive Nav2 map and generate goals on first map reception."""
        self.get_logger().info(f'Map callback called! Map is None: {self.map is None}')
        if self.map is None:
            self.map = msg
            self.get_logger().info('Map received, generating goals...')
            self.generate_goals()
            self.get_logger().info(f'Goals generated: {len(self.goals)}')
 
    def is_free(self, x, y):
        """Check if a position is free using the Nav2 OccupancyGrid map.
        
        OccupancyGrid cells:
        0   = free
        100 = obstacle
        -1  = unknown
        """
        if self.map is None:
            return True  # if map not available yet, accept the goal
 
        # Convert world coordinates to grid cell indices
        mx = int((x - self.map.info.origin.position.x) / self.map.info.resolution)
        my = int((y - self.map.info.origin.position.y) / self.map.info.resolution)
 
        # Check bounds
        if mx < 0 or my < 0 or mx >= self.map.info.width or my >= self.map.info.height:
            return False
 
        # Check if cell is free (0 = free)
        idx = my * self.map.info.width + mx
        return self.map.data[idx] == 0
 
    def is_valid_goal(self, x, y):
        """Check if a goal position is valid:
        - Must be on a free cell (not obstacle)
        - Must not be too close to existing goals
        """
        # Check if position is free on the map
        if not self.is_free(x, y):
            return False
 
        # Check if the position is not too close to existing goals
        for goal in self.goals:
            dx = x - goal['x']
            dy = y - goal['y']
            if math.sqrt(dx**2 + dy**2) < MIN_GOAL_DISTANCE:
                return False
 
        return True
 
    def generate_goals(self):
        """Generate NUM_GOALS random positions in the arena on free cells."""
        self.goals = []
        attempts = 0
        while len(self.goals) < NUM_GOALS and attempts < 1000:
            x = random.uniform(ARENA_X_MIN, ARENA_X_MAX)
            y = random.uniform(ARENA_Y_MIN, ARENA_Y_MAX)
            if self.is_valid_goal(x, y):
                self.goals.append({'id': len(self.goals), 'x': x, 'y': y, 'active': True})
            attempts += 1
 
    def robot1_pose_callback(self, msg):
        self.robot1_pose = msg.pose.pose
 
    def robot2_pose_callback(self, msg):
        self.robot2_pose = msg.pose.pose
 
    def distance(self, pose, goal):
        """Calculate Euclidean distance between pose and goal."""
        dx = pose.position.x - goal['x']
        dy = pose.position.y - goal['y']
        return math.sqrt(dx**2 + dy**2)
 
    def check_goals(self):
        """Check if a robot has reached a goal."""
        for goal in self.goals:
            if not goal['active']:
                continue
 
            if self.robot1_pose:
                if self.distance(self.robot1_pose, goal) < GOAL_RADIUS:
                    goal['active'] = False
                    self.score['robot1'] += 1
                    self.get_logger().info(
                        f'Robot1 has reached goal {goal["id"]}! Score: {self.score}')
 
            if self.robot2_pose and goal['active']:
                if self.distance(self.robot2_pose, goal) < GOAL_RADIUS:
                    goal['active'] = False
                    self.score['robot2'] += 1
                    self.get_logger().info(
                        f'Robot2 has reached goal {goal["id"]}! Score: {self.score}')
 
    def check_winner(self):
        """Check if the game is over."""
        active_goals = [g for g in self.goals if g['active']]

        if len(self.goals) == 0:
            return  # goals not generated yet

        if len(active_goals) == 0:
            self.game_over = True
            if self.score['robot1'] > self.score['robot2']:
                winner = 'robot1'
            elif self.score['robot2'] > self.score['robot1']:
                winner = 'robot2'
            else:
                winner = 'pareggio'
            self.get_logger().info(f'GAME OVER! Winner: {winner}')
            self.get_logger().info(f'Final score: {self.score}')
 
    def publish_goals(self):
        """Publish the list of active goals."""
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
 
        for goal in self.goals:
            if goal['active']:
                pose = Pose()
                pose.position.x = goal['x']
                pose.position.y = goal['y']
                pose.position.z = 0.0
                msg.poses.append(pose)
 
        self.goals_pub.publish(msg)
 
    def publish_score(self):
        """Publish the current score."""
        score_data = {
            'robot1': self.score['robot1'],
            'robot2': self.score['robot2'],
            'game_over': self.game_over
        }
        msg = String()
        msg.data = json.dumps(score_data)
        self.score_pub.publish(msg)
 
    def game_loop(self):
        """Main game loop."""
        if self.game_over:
            return
        
        if len(self.goals) == 0:
            return  # waiting for map

        self.check_goals()
        self.check_winner()
        self.publish_goals()
        self.publish_score()
 
 
def main(args=None):
    rclpy.init(args=args)
    node = GameMaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()