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
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration

# Game parameters
NUM_GOALS = 10
GOAL_RADIUS = 0.3  # Generoso per la fluidità
ARENA_X_MIN = -2.9
ARENA_X_MAX = 2.9
ARENA_Y_MIN = -3.9
ARENA_Y_MAX = 3.9
PUBLISH_RATE = 1.0  # Hz
MIN_GOAL_DISTANCE = GOAL_RADIUS * 4  # minimum distance between goals
OBSTACLE_MARGIN = 0.35  # Margine di sicurezza fisico dagli ostacoli

class GameMaster(Node):
 
    def __init__(self):
        super().__init__('game_master')
 
        qos = QoSProfile(depth=10)

        # Publisher
        self.goals_pub = self.create_publisher(PoseArray, '/game/goals', qos)
        self.score_pub = self.create_publisher(String, '/game/score', qos)
        self.marker_pub = self.create_publisher(MarkerArray, '/game/goal_markers', qos) 

        # Subscriber robot position
        self.create_subscription(PoseWithCovarianceStamped, '/robot1/amcl_pose', self.robot1_pose_callback, qos)
        self.create_subscription(PoseWithCovarianceStamped, '/robot2/amcl_pose', self.robot2_pose_callback, qos)
 
        # Play state
        self.robot1_pose = None
        self.robot2_pose = None
        self.score = {'robot1': 0, 'robot2': 0}
        self.goals = []
        self.game_over = False
 
        # GENERAZIONE IMMEDIATA: Non aspettiamo più nessuna mappa!
        self.generate_goals()
        self.get_logger().info(f'Goals generated: {len(self.goals)}')

        # Pubblication timer
        self.create_timer(1.0 / PUBLISH_RATE, self.game_loop)
        self.get_logger().info('Game Master started!')

    def is_valid_goal(self, x, y):
        """Check if a goal position is valid using Absolute Mathematical Truth."""
        margin = OBSTACLE_MARGIN

        # 1. Limiti Esterni
        if x < (-3.0 + margin) or x > (3.0 - margin) or y < (-4.0 + margin) or y > (4.0 - margin):
            return False

        # 2. Muro Divisorio Centrale (Pose: 1.0, -1.0 | Size: 4.0 x 0.05)
        if (-1.0 - margin) <= x <= (3.0 + margin) and (-1.025 - margin) <= y <= (-0.975 + margin):
            return False

        # 3. Big Box Obstacle (Pose: -1.0, 1.5 | Size: 2.0 x 3.0)
        if (-2.0 - margin) <= x <= (0.0 + margin) and (0.0 - margin) <= y <= (3.0 + margin):
            return False

        # 4. Small Box Obstacle (Pose: -1.0, -2.5 | Size: 2.0 x 1.0)
        if (-2.0 - margin) <= x <= (0.0 + margin) and (-3.0 - margin) <= y <= (-2.0 + margin):
            return False

        # 5. Cilindri (Centri: [1.5, 2.5] e [1.5, 0.5])
        cylinder_radius = 0.5 + margin 
        if math.sqrt((x - 1.5)**2 + (y - 2.5)**2) <= cylinder_radius:
            return False
        if math.sqrt((x - 1.5)**2 + (y - 0.5)**2) <= cylinder_radius:
            return False

        # 6. Punti di Spawn (Niente goal gratuiti alla partenza)
        if math.sqrt((x - 2.5)**2 + (y - (-1.5))**2) < 0.6:
            return False
        if math.sqrt((x - 2.5)**2 + (y - (-2.5))**2) < 0.6:
            return False

        # 7. Check if the position is not too close to existing goals
        for goal in self.goals:
            dx = x - goal['x']
            dy = y - goal['y']
            if math.sqrt(dx**2 + dy**2) < MIN_GOAL_DISTANCE:
                return False
 
        return True
 
    def generate_goals(self):
        """Generate NUM_GOALS random positions mathematically guaranteed to be free."""
        self.goals = []
        attempts = 0
        # Aumentato il numero di tentativi perché la matematica è velocissima
        while len(self.goals) < NUM_GOALS and attempts < 2000:
            x = random.uniform(ARENA_X_MIN, ARENA_X_MAX)
            y = random.uniform(ARENA_Y_MIN, ARENA_Y_MAX)
            if self.is_valid_goal(x, y):
                self.goals.append({'id': len(self.goals), 'x': x, 'y': y, 'active': True, 'collected_by': None})
            attempts += 1
 
    def robot1_pose_callback(self, msg):
        self.robot1_pose = msg.pose.pose
 
    def robot2_pose_callback(self, msg):
        self.robot2_pose = msg.pose.pose
 
    def distance(self, pose, goal):
        dx = pose.position.x - goal['x']
        dy = pose.position.y - goal['y']
        return math.sqrt(dx**2 + dy**2)
 
    def check_goals(self):
        for goal in self.goals:
            if not goal['active']:
                continue
 
            if self.robot1_pose:
                if self.distance(self.robot1_pose, goal) < GOAL_RADIUS:
                    goal['active'] = False
                    self.score['robot1'] += 1
                    goal['collected_by'] = 'robot1' 
                    self.get_logger().info(f'Robot1 has reached goal {goal["id"]}! Score: {self.score}')
 
            if self.robot2_pose and goal['active']:
                if self.distance(self.robot2_pose, goal) < GOAL_RADIUS:
                    goal['active'] = False
                    self.score['robot2'] += 1
                    goal['collected_by'] = 'robot2' 
                    self.get_logger().info(f'Robot2 has reached goal {goal["id"]}! Score: {self.score}')
 
    def check_winner(self):
        active_goals = [g for g in self.goals if g['active']]
        if len(self.goals) == 0:
            return

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
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        marker_array = MarkerArray()
 
        for goal in self.goals:
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = msg.header.stamp
            marker.ns = 'shared_arena_goals'
            marker.id = goal['id']
            
            pose = Pose()
            pose.position.x = goal['x']
            pose.position.y = goal['y']
            pose.position.z = 0.0
            
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = goal['x']
            marker.pose.position.y = goal['y']
            marker.pose.position.z = 0.01
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.4
            marker.scale.y = 0.4
            marker.scale.z = 0.01

            if goal['active']:
                msg.poses.append(pose)
                marker.color.a = 0.9  
                marker.color.r = 0.0  
                marker.color.g = 1.0  
                marker.color.b = 0.0  
            else:
                if goal['collected_by'] == 'robot1':
                    marker.color.a = 0.9 
                    marker.color.r = 0.0  
                    marker.color.g = 0.0  
                    marker.color.b = 1.0 
                elif goal['collected_by'] == 'robot2':
                    marker.color.a = 0.9
                    marker.color.r = 1.0  
                    marker.color.g = 0.0  
                    marker.color.b = 0.9

            marker.lifetime = Duration(sec=2, nanosec=0)
            marker_array.markers.append(marker)

        self.goals_pub.publish(msg)
        self.marker_pub.publish(marker_array)
 
    def publish_score(self):
        score_data = {
            'robot1': self.score['robot1'],
            'robot2': self.score['robot2'],
            'game_over': self.game_over
        }
        msg = String()
        msg.data = json.dumps(score_data)
        self.score_pub.publish(msg)
 
    def game_loop(self):
        if self.game_over or len(self.goals) == 0:
            return
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
