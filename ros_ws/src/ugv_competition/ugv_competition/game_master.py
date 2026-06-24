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
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy # Added ReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose
from std_msgs.msg import String
import random
import math
import json
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
from nav_msgs.msg import OccupancyGrid

# Game parameters
NUM_GOALS = 10
GOAL_RADIUS = 0.2  
ARENA_X_MIN = -2.8  
ARENA_X_MAX = 2.8
ARENA_Y_MIN = -3.8
ARENA_Y_MAX = 3.8
PUBLISH_RATE = 5.0  # Hz
MIN_GOAL_DISTANCE = GOAL_RADIUS * 4  
OBSTACLE_MARGIN = 0.35  
SPAWN_MARGIN = 1.0  # Minimum distance (meters) from robot spawn points

class GameMaster(Node):
    def __init__(self):
        super().__init__('game_master')
 
        qos = QoSProfile(depth=10)
        amcl_qos = QoSProfile(depth=10)
        amcl_qos.reliability = ReliabilityPolicy.RELIABLE
        amcl_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        # New parameter used to control if the goals are generated in a random or simmetric way wrt the map
        # Simmetric way was usefull to test different metrics
        self.declare_parameter('goal_placement', 'random')
        self.goal_placement = self.get_parameter('goal_placement').value

        # Goal seed for reproducibility
        self.declare_parameter('goal_seed', -1)
        goal_seed = self.get_parameter('goal_seed').value

        if goal_seed >= 0:
            random.seed(goal_seed)

        
        # Publishers
        self.goals_pub = self.create_publisher(PoseArray, '/game/goals', qos)
        self.score_pub = self.create_publisher(String, '/game/score', qos)
        self.marker_pub = self.create_publisher(MarkerArray, '/game/goal_markers', qos) 
        self.banner_pub = self.create_publisher(Marker, '/game/banner', qos)

        # Subscribers for robot positions
        self.create_subscription(
            PoseWithCovarianceStamped, '/robot1/amcl_pose',
              self.robot1_pose_callback, amcl_qos)
        
        self.create_subscription(
             PoseWithCovarianceStamped, '/robot2/amcl_pose',
             self.robot2_pose_callback, amcl_qos)
    
        # State variables
        self.robot1_pose = None
        self.robot2_pose = None
        self.robot1_spawn = None  # Stores the very first pose received
        self.robot2_spawn = None  # Stores the very first pose received
        self.winner = None
        
        self.score = {'robot1': 0, 'robot2': 0}
        self.goals = []
        self.game_over = False
        self.map = None
        self.map_received = False
        self.goals_generated = False  # Flag to prevent regenerating goals
 
        # Map subscriber (Transient Local to get the latched map)
        map_qos = QoSProfile(depth=10)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL 
        self.create_subscription(OccupancyGrid, '/robot1/map', self.map_callback, map_qos) #same if i use robot2

        # Game loop timer
        self.create_timer(1.0 / PUBLISH_RATE, self.game_loop)
        self.get_logger().info('Game Master started! Waiting for Map and Robot Poses...')

    def check_and_generate_goals(self):
        """Synchronizes goal generation: waits for Map + Robot1 Pose + Robot2 Pose."""
        if not self.goals_generated:
            if self.map_received and self.robot1_spawn is not None and self.robot2_spawn is not None:
                self.get_logger().info('Map and initial robot poses received! Generating goals in C_free...')
                self.generate_goals()
                self.goals_generated = True
            else:
                missing = []
                if not self.map_received: missing.append('Map')
                if self.robot1_spawn is None: missing.append('Robot1 AMCL pose')
                if self.robot2_spawn is None: missing.append('Robot2 AMCL pose')
                self.get_logger().info(f'Waiting for: {", ".join(missing)}')

    def map_callback(self, msg):
        if not self.map_received:
            self.map = msg
            self.map_received = True
            self.get_logger().info('>>> MAP RECEIVED! <<<')
            self.check_and_generate_goals()

    def robot1_pose_callback(self, msg):
        pose = msg.pose.pose
        if self.robot1_spawn is None:
            self.robot1_spawn = pose
            self.get_logger().info('>>> ROBOT1 INITIAL POSE RECEIVED! <<<')
            self.check_and_generate_goals()
        self.robot1_pose = pose  # Keep updating for the game loop
 
    def robot2_pose_callback(self, msg):
        pose = msg.pose.pose
        if self.robot2_spawn is None:
            self.robot2_spawn = pose
            self.get_logger().info('>>> ROBOT2 INITIAL POSE RECEIVED! <<<')
            self.check_and_generate_goals()
        self.robot2_pose = pose
 
    def is_free(self, x, y):
        """Check if a position is free using the Nav2 OccupancyGrid map."""
        if self.map is None:
            return False  # Must have map to verify
 
        width = self.map.info.width
        height = self.map.info.height
        resolution = self.map.info.resolution
        origin_x = self.map.info.origin.position.x
        origin_y = self.map.info.origin.position.y

        mx = int((x - origin_x) / resolution)
        my = int((y - origin_y) / resolution)
        margin_cells = int(OBSTACLE_MARGIN / resolution)

        # Scan a circular area around the central point
        for dx in range(-margin_cells, margin_cells + 1):
            for dy in range(-margin_cells, margin_cells + 1):
                if math.hypot(dx, dy) > margin_cells:
                    continue

                cx = mx + dx
                cy = my + dy

                if cx < 0 or cy < 0 or cx >= width or cy >= height:
                    return False  # Outside map boundaries
                
                idx = cy * width + cx
                # 0 = free, 100 = obstacle, -1 = unknown. We treat unknown as obstacle for safety.
                if self.map.data[idx] != 0:
                    return False  
 
        return True
 
    def is_valid_goal(self, x, y):
        """Unified validation: relies purely on the Map (is_free) + Distance checks."""
        
        # 1. Check if it's physically free in the OccupancyGrid (handles ALL obstacles automatically)
        if not self.is_free(x, y):
            return False

        # 2. Check distance to existing goals
        for goal in self.goals:
            if math.hypot(x - goal['x'], y - goal['y']) < MIN_GOAL_DISTANCE:
                return False

        # 3. Check distance to robot SPAWN positions (prevents spawning on top of robots)
        if self.robot1_spawn:
            if math.hypot(x - self.robot1_spawn.position.x, y - self.robot1_spawn.position.y) < SPAWN_MARGIN:
                return False
                
        if self.robot2_spawn:
            if math.hypot(x - self.robot2_spawn.position.x, y - self.robot2_spawn.position.y) < SPAWN_MARGIN:
                return False
 
        return True

    def generate_goals_symmetric(self):
        """Generate NUM_GOALS goals placed symmetrically around the arena center.
        Half the goals are placed in the top zone (y > center_y) and half in the 
        bottom zone (y < center_y), mirrored around the center point.
        """
        self.goals = []
        center_x = (ARENA_X_MIN + ARENA_X_MAX) / 2
        center_y = (ARENA_Y_MIN + ARENA_Y_MAX) / 2
        
        attempts = 0
        half = NUM_GOALS // 2
        
        while len(self.goals) < half and attempts < 5000:
            x = random.uniform(ARENA_X_MIN, ARENA_X_MAX)
            y = random.uniform(center_y + 0.3, ARENA_Y_MAX)
            mirror_x = 2 * center_x - x
            mirror_y = 2 * center_y - y
            
            # Only add the pair if BOTH positions are valid
            if self.is_valid_goal(x, y) and self.is_valid_goal(mirror_x, mirror_y):
                self.goals.append({'id': len(self.goals), 'x': x, 'y': y,
                                'active': True, 'collected_by': None})
                self.goals.append({'id': len(self.goals), 'x': mirror_x, 'y': mirror_y,
                                'active': True, 'collected_by': None})
            attempts += 1
        
        if len(self.goals) < NUM_GOALS:
            self.get_logger().warn(f'Could only generate {len(self.goals)} symmetric goals.')
            
    def generate_goals_random(self):
        """Generate NUM_GOALS random positions mathematically guaranteed to be free."""
        self.goals = []
        attempts = 0
        while len(self.goals) < NUM_GOALS and attempts < 5000:
            x = random.uniform(ARENA_X_MIN, ARENA_X_MAX)
            y = random.uniform(ARENA_Y_MIN, ARENA_Y_MAX)
            if self.is_valid_goal(x, y):
                self.goals.append({'id': len(self.goals), 'x': x, 'y': y, 'active': True, 'collected_by': None})
            attempts += 1
            
        if len(self.goals) < NUM_GOALS:
            self.get_logger().warn(f'Could only generate {len(self.goals)} out of {NUM_GOALS} goals. Map might be too cluttered.')
    
    def generate_goals(self):
        """ 
        Select how to generate goals: symmetric or random 
        """
        if self.goal_placement == 'symmetric':
            self.generate_goals_symmetric()
        else:
            self.generate_goals_random()

    def distance(self, pose, goal):
        dx = pose.position.x - goal['x']
        dy = pose.position.y - goal['y']
        return math.hypot(dx, dy)
 
    def check_goals(self):
        for goal in self.goals:
            if not goal['active']:
                continue
 
            if self.robot1_pose and self.distance(self.robot1_pose, goal) < GOAL_RADIUS:
                goal['active'] = False
                self.score['robot1'] += 1
                goal['collected_by'] = 'robot1' 
                self.get_logger().info(f'Robot1 reached goal {goal["id"]}! Score: {self.score}')
 
            if self.robot2_pose and goal['active'] and self.distance(self.robot2_pose, goal) < GOAL_RADIUS:
                goal['active'] = False
                self.score['robot2'] += 1
                goal['collected_by'] = 'robot2' 
                self.get_logger().info(f'Robot2 reached goal {goal["id"]}! Score: {self.score}')
 
    def check_winner(self):
        active_goals = [g for g in self.goals if g['active']]
        if len(active_goals) == 0 and len(self.goals) > 0:
            self.game_over = True
            if self.score['robot1'] > self.score['robot2']:
                self.winner = 'robot1'
            elif self.score['robot2'] > self.score['robot1']:
                self.winner = 'robot2'
            else:
                self.winner = 'pareggio'
            self.get_logger().info(f'GAME OVER! Winner: {self.winner} | Final score: {self.score}')
 
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
                marker.color.a, marker.color.r, marker.color.g, marker.color.b = 0.9, 0.0, 1.0, 0.0
            else:
                if goal['collected_by'] == 'robot1':
                    marker.color.a, marker.color.r, marker.color.g, marker.color.b = 0.9, 0.0, 0.0, 1.0
                elif goal['collected_by'] == 'robot2':
                    marker.color.a, marker.color.r, marker.color.g, marker.color.b = 0.9, 1.0, 0.0, 0.0

            marker.lifetime = Duration(sec=2, nanosec=0)
            marker_array.markers.append(marker)

        self.goals_pub.publish(msg)
        self.marker_pub.publish(marker_array)
 
    def publish_score(self):
        score_data = {'robot1': self.score['robot1'], 'robot2': self.score['robot2'], 'game_over': self.game_over}
        msg = String()
        msg.data = json.dumps(score_data)
        self.score_pub.publish(msg)
 
    def game_loop(self):
     if self.game_over:
        if self.winner is not None:
            self.publish_winner_banner(self.winner)
        return

     if not self.goals_generated:
        return

     self.check_goals()
     self.check_winner()
     self.publish_goals()
     self.publish_score()

    def publish_winner_banner(self, winner: str):
        """Publish a large text marker visible in RViz at the center of the arena"""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'game_banner'
        marker.id = 999
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

      # Position it above the center of the arena
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 1.5      # float above the ground
        marker.pose.orientation.w = 1.0

        marker.scale.z = 0.4              # text height in meters

    # Color: gold for a winner, white for a draw
        if winner == 'pareggio':
           marker.color.r = 1.0
           marker.color.g = 1.0
           marker.color.b = 1.0
           marker.color.a = 1.0 
        else:
           marker.color.r = 1.0
           marker.color.g = 0.84
           marker.color.b = 0.0
           marker.color.a = 1.0

        if winner == 'robot1':
            marker.text = " ROBOT 1 WINS! \nFinal: R1={} R2={}".format(self.score['robot1'], self.score['robot2'])
        elif winner == 'robot2':
            marker.text = " ROBOT 2 WINS! \nFinal: R1={} R2={}".format(self.score['robot1'], self.score['robot2'])
        else:
            marker.text = " IT'S A DRAW!\nFinal: R1={} R2={}".format(self.score['robot1'], self.score['robot2'])

       # Publish repeatedly so it stays visible
        self.banner_pub.publish(marker)
 
def main(args=None):
    rclpy.init(args=args)
    node = GameMaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()

