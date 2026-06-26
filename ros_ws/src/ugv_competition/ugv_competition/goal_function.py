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
from nav_msgs.msg import OccupancyGrid  # Added for wall detection
from geometry_msgs.msg import Twist # Needed to handle deadlock
from tf_transformations import euler_from_quaternion #Needed to transform from quaternions to euler
import math
import json
from ugv_competition.metrics import metrics as metric_module
from enum import Enum, auto

class NavState(Enum):
    IDLE        = auto()  # no goal,ready to receive it
    SENDING     = auto()  # send_goal_async sended,waiting for a response from Nav2
    NAVIGATING  = auto()  # goal accepted,robot is moving
    CANCELLING  = auto()  # cancel_goal_async sended,waiting for confirm
    RECOVERING    = auto()  # recover from a blocking state

# Strategy parameters
ALPHA = 3.0   # weight for own distance (higher = prefer closer goals)
BETA = 4.0    # weight for competitive advantage (higher = prefer blocking opponent)
GAMMA = 1.0 
GOAL_REACHED_THRESHOLD = 0.30  # meters
MAX_GOAL_DISTANCE = 3.0  # max distance to consider a goal reachable
MAX_DISTANCE_ARENA = 10.0  # max distance in the arena for goal selection
WALL_PENALTY = 15.0  # Heavy penalty (in meters) added if a wall blocks the path to the goal
MIN_GOAL_DURATION = 2.0    # min seconds to change goal
SWITCH_THRESHOLD = 0.3     # min margin to change goal

class GoalFunction(Node):

    def __init__(self):
        super().__init__('goal_function')

        # Robot name parameter (robot1 or robot2)
        self.declare_parameter('robot_name', 'robot1')
        self.robot_name = self.get_parameter('robot_name').value

        # Metric selection parameter
        self.declare_parameter('metric_name', 'euclidean')
        self.metric_name = self.get_parameter('metric_name').value
        
        
        # Opponent name
        self.opponent_name = 'robot2' if self.robot_name == 'robot1' else 'robot1'

        # Use namespace if robot_name is set
        self.use_namespace = self.declare_parameter('use_namespace', True).value

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

        amcl_qos = QoSProfile(depth=10)
        amcl_qos.reliability = ReliabilityPolicy.RELIABLE
        amcl_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL


        self.create_subscription(
            PoseWithCovarianceStamped,
            own_pose_topic,
            self.own_pose_callback,
            amcl_qos)

        # Subscribe to opponent pose
        self.create_subscription(
            PoseWithCovarianceStamped,
            opponent_pose_topic,
            self.opponent_pose_callback,
            amcl_qos)

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
        
        # Subscribe to the Map to detect walls
        map_qos = QoSProfile(depth=10)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(OccupancyGrid, '/robot1/map', self.map_callback, map_qos) #same if i sue robot2
        self.map = None

        # Velocity publisher, to handle deadlock
        self.cmd_vel_pub = self.create_publisher(Twist, f'/{self.robot_name}/cmd_vel', 10)
        # Variables to handle deadlock
        self.last_position = None
        self.last_position_time = None
        self.stuck_timeout = 12.0  # seconds

        # State
        self.goals = []
        self.own_pose = None
        self.opponent_pose = None
        self.current_goal = None
        self.game_over = False
        self.own_goal_handle = None
        self.pending_goal = None 
        self.last_goal_sent_time = 0.0
        self.last_attempted_goal = None
        self.nav_state = NavState.IDLE
        

        # Timer for goal selection
        self.create_timer(1, self.select_and_send_goal) 

        self.get_logger().info(f'{self.robot_name} Goal Function started!')
        self.get_logger().info(f'Own pose topic: {own_pose_topic}')
        self.get_logger().info(f'Nav2 action: {nav2_action}')

        # Map string names to actual functions from the metrics module
        self.metric_functions = {
            'euclidean': metric_module.euclidean_distance,
            'manhattan': metric_module.manhattan_distance,
            'estimated_time': metric_module.estimated_time_to_goal,
            'cluster': metric_module.cluster_score
        }
        
        if self.metric_name not in self.metric_functions:
            self.get_logger().error(f"Unknown metric '{self.metric_name}'. Defaulting to 'euclidean'.")
            self.metric_name = 'euclidean'
            
        self.get_logger().info(f'Using metric: {self.metric_name}')


      

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

    def map_callback(self, msg):
        """Receive the OccupancyGrid map to check for walls"""
        self.map = msg

    def get_metric_cost(self, pose, goal):
        """Dynamically calls the selected metric function to calculate the cost/distance."""
        
        return self.metric_functions[self.metric_name](pose, goal)

    

    def bresenham_line(self, start_x, start_y, end_x, end_y):
        """
        Casts a virtual ray from start to end on the OccupancyGrid.
        Returns True if the path is clear, False if a wall blocks it.
        Uses Bresenham's Line Algorithm.
        """
        
    
        if self.map is None:
            self.get_logger().warn('Map not received yet! Assuming clear path.')
            return True  # If no map yet, assume clear
        


        resolution = self.map.info.resolution
        origin_x = self.map.info.origin.position.x
        origin_y = self.map.info.origin.position.y
        width = self.map.info.width
        height = self.map.info.height

        # Convert world coordinates to grid cell indices
        x0 = int((start_x - origin_x) / resolution)
        y0 = int((start_y - origin_y) / resolution)
        x1 = int((end_x - origin_x) / resolution)
        y1 = int((end_y - origin_y) / resolution)

        # Bresenham's line algorithm variables
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while True:
            # Check if current cell is within map bounds
            if not (0 <= x0 < width and 0 <= y0 < height):
                return False  # Ray went out of bounds
                
            # Check if the cell is an obstacle (0 = free, >0 = wall/obstacle, -1 = unknown)
            # We only block on known walls (>0) to avoid localization noise blinding the robot
            idx = y0 * width + x0
            if self.map.data[idx] > 25: #to avoid artifacts or noise
                return False  # Blocked by a wall!

            if x0 == x1 and y0 == y1:
                break  # Reached the goal
                
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy

        return True  # Ray reached the goal without hitting a wall

    def is_stuck(self):
        """
        Function which controls if the robot is stuck in a position, because of the deadlock
        """
        if self.own_pose is None:
            return False
        now = self.get_clock().now().nanoseconds / 1e9
        current_pos = (self.own_pose.position.x, self.own_pose.position.y)
        
        if self.last_position is None:
            self.last_position = current_pos
            self.last_position_time = now
            return False
        
        dist = math.hypot(current_pos[0] - self.last_position[0],
                        current_pos[1] - self.last_position[1])
        
        if dist > 0.05:  # robot has moved, reset stuck detection
            self.last_position = current_pos
            self.last_position_time = now
            return False
        
        return (now - self.last_position_time) > self.stuck_timeout

    def handle_stuck(self):
        """Force the robot to back up when stuck."""
        self.get_logger().warn('Robot is stuck! Backing up...')
        
        self.nav_state=NavState.RECOVERING

        # Cancel current Nav2 goal to stop conflicting cmd_vel
        if self.own_goal_handle is not None:
            self.own_goal_handle.cancel_goal_async()

        self.current_goal = None
        self.own_goal_handle = None
        self.last_position = None  # reset stuck detection
        
        # Publish backup command for 2 seconds using a timer, to not block ROS2
        self.stuck_counter = 0
        self.stuck_timer = self.create_timer(0.1, self._backup_callback)
        self.stuck_timer.reset() 

    def _backup_callback(self):
        """Callback to publish backup command for 2 seconds."""
        twist = Twist()
        twist.linear.x = -0.1
        twist.angular.z = 0.5
        self.cmd_vel_pub.publish(twist)
        self.stuck_counter += 1
        if self.stuck_counter >= 20:  # 20 * 0.1s = 2 seconds
            self.stuck_timer.cancel()
            self.stuck_counter = 0
            self.nav_state=NavState.IDLE

    def score_goal(self, goal):
        """Calculate the score for a goal using greedy strategy + Wall Penalties."""
        if self.own_pose is None:
            return -9999.0

        own_dist = self.get_metric_cost(self.own_pose, goal)

        score = 0

        # Check if a wall is in the way 
        if not self.bresenham_line(self.own_pose.position.x, self.own_pose.position.y, goal['x'], goal['y']):
            # if a wall blocks the straight path add a heavy penalty.
            # This forces the robot to finish all goals on its own side before crossing the wall.
            score -= WALL_PENALTY
            #own_dist += WALL_PENALTY

        if self.opponent_pose is not None:
            opp_dist = self.get_metric_cost(self.opponent_pose, goal)

            competitive_advantage = opp_dist - own_dist
        else:
            competitive_advantage = 0.0  
        
        score += -ALPHA * own_dist + BETA * competitive_advantage + GAMMA * metric_module.cluster_score(goal, self.goals)

        return score
    
    def select_best_goal(self):
        """Select the best goal using the greedy scoring function."""
        new_radius = MAX_GOAL_DISTANCE

        if not self.goals or self.own_pose is None: 
            return None
    
        
        reachable_goals = []
        
        #If NO goals are within 1 meter, add 1 meter to the radius
        while not reachable_goals and new_radius <= MAX_DISTANCE_ARENA:
            #check goals with current distance
            for g in self.goals:
                dist = self.get_metric_cost(self.own_pose, g)
                if dist <= new_radius:
                    reachable_goals.append(g)
            
            # If no goals are within radius, expand by 1 meter
            if not reachable_goals:
                new_radius += 2.0
        
        if not reachable_goals:
            return None

        best_goal = max(reachable_goals, key=lambda g: self.score_goal(g))
        return best_goal



    def is_current_goal_still_active(self):
        """Check if the current goal is still in the active goals list."""
        if self.current_goal is None:
            return False
        for g in self.goals:
            if abs(g['x'] - self.current_goal['x']) < 0.01 and \
            abs(g['y'] - self.current_goal['y']) < 0.01:
                return True
        return False

    def is_goal_reached(self):
        """Check if the current goal has been reached."""
        if self.current_goal is None or self.own_pose is None:
            return False
        return metric_module.euclidean_distance(self.own_pose, self.current_goal) <= GOAL_REACHED_THRESHOLD
    

    def is_same_goal(self, g1, g2):
        """Helper to check if two goals are geometrically identical (within 1cm)"""
        if g1 is None or g2 is None:
            return False
        return math.hypot(g1['x'] - g2['x'], g1['y'] - g2['y']) < 0.01

    def should_switch_goal(self, best_goal):
        # Prevent switching goals too frequently by enforcing a minimum time between goal changes
        now = self.get_clock().now().nanoseconds / 1e9
        if self.last_goal_sent_time and (now - self.last_goal_sent_time) < MIN_GOAL_DURATION:
            return False

        best_score = self.score_goal(best_goal)
        current_score = self.score_goal(self.current_goal)
    
        # Adaptive threshold that increases as we get closer to the current goal, to prevent oscillations near the goal
        dist_to_current = metric_module.euclidean_distance(self.own_pose, self.current_goal)
        adaptive_threshold = SWITCH_THRESHOLD + (1.0 / (dist_to_current + 0.1))
        return best_score > current_score + adaptive_threshold
    

    






    def send(self, goal):
        self.nav_state = NavState.SENDING      
        self.current_goal = goal
        self.last_attempted_goal=goal
        self.last_goal_sent_time = self.get_clock().now().nanoseconds / 1e9

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = goal['x']
        goal_msg.pose.pose.position.y = goal['y']
        goal_msg.pose.pose.orientation.w = 1.0

        future = self.nav2_client.send_goal_async(goal_msg)
        future.add_done_callback(self.on_goal_response)

        self.get_logger().info(f'{self.robot_name} navigating to ({goal["x"]:.2f}, {goal["y"]:.2f})')

    def on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.nav_state = NavState.IDLE       
            self.current_goal = None
            return
        self.own_goal_handle = handle
        self.nav_state = NavState.NAVIGATING     
        handle.get_result_async().add_done_callback(self._on_goal_result)

    def cancel_and_send(self, goal):
        self.nav_state = NavState.CANCELLING     
        self.pending_goal = goal
        self.own_goal_handle.cancel_goal_async().add_done_callback(self.on_cancel_done)

    def on_cancel_done(self, future):
        goal_to_send = self.pending_goal
        self.pending_goal = None
        self.own_goal_handle = None
        if goal_to_send:
            self.send(goal_to_send)             
        else:
            self.nav_state = NavState.IDLE

    def _on_goal_result(self, future):
        if self.nav_state in [NavState.CANCELLING, NavState.RECOVERING, NavState.SENDING]:
            return  # il cancel gestirà lui la prossima transizione
        self.own_goal_handle = None
        self.current_goal = None
        self.nav_state = NavState.IDLE           

    def select_and_send_goal(self):

        # Nel timer select_and_send_goal, all'inizio:
        now = self.get_clock().now().nanoseconds / 1e9
        if self.nav_state == NavState.SENDING:
            if now - self.last_goal_sent_time > 5.0:
                self.get_logger().warn('Nav2 non ha risposto in 5s, reset a IDLE')
                self.nav_state = NavState.IDLE
                self.current_goal = None
            return
        
        self.get_logger().info(f'Goals: {len(self.goals)}, Own pose: {self.own_pose is not None}')
        """Main loop: select best goal and send to Nav2."""
        
        if self.game_over or not self.goals or self.own_pose is None:
            return
        
        if self.nav_state != NavState.IDLE and self.nav_state != NavState.NAVIGATING:
            return  # SENDING,RECOVERING or CANCELLING: wait until the state change
        
        if self.nav_state==NavState.NAVIGATING and self.is_stuck():
            self.handle_stuck()
            return

        best_goal = self.select_best_goal()
        if best_goal is None:
            return

        if self.nav_state == NavState.IDLE:
            if self.is_same_goal(best_goal, self.last_attempted_goal) and (now - self.last_goal_sent_time < 5.0):
                return
            self.send(best_goal)

        elif self.nav_state == NavState.NAVIGATING:
            if self.is_same_goal(best_goal, self.current_goal):
                return
            if not self.is_current_goal_still_active():
                self.cancel_and_send(best_goal)
            elif self.is_goal_reached():
                self.cancel_and_send(best_goal)
            elif self.should_switch_goal(best_goal):
                self.cancel_and_send(best_goal)




def main(args=None):
    rclpy.init(args=args)
    node = GoalFunction()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

