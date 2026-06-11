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
import math
import json

# Strategy parameters
ALPHA = 1.0   # weight for own distance (higher = prefer closer goals)
BETA = 1.5    # weight for competitive advantage (higher = prefer blocking opponent)
GOAL_REACHED_THRESHOLD = 0.10  # meters
MAX_GOAL_DISTANCE = 3.0  # max distance to consider a goal reachable
MAX_DISTANCE_ARENA = 10.0  # max distance in the arena for goal selection
WALL_PENALTY = 2.0  # Heavy penalty (in meters) added if a wall blocks the path to the goal
MIN_GOAL_DURATION = 2.0    # min seconds to change goal
SWITCH_THRESHOLD = 0.3     # min margin to change goal

class GoalFunction(Node):

    def __init__(self):
        super().__init__('goal_function')

        # Robot name parameter (robot1 or robot2)
        self.declare_parameter('robot_name', 'robot1')
        self.robot_name = self.get_parameter('robot_name').value

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


        # State
        self.goals = []
        self.own_pose = None
        self.opponent_pose = None
        self.current_goal = None
        self.game_over = False
        self.navigating = False
        self.own_goal_handle = None
        self.pending_goal = None 
        self.last_goal_sent_time = 0.0

        # Timer for goal selection
        self.create_timer(1, self.select_and_send_goal) 

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

    def map_callback(self, msg):
        """Receive the OccupancyGrid map to check for walls"""
        self.map = msg

    def euclidean_distance(self, pose, goal):
        """Calculate Euclidean distance between a pose and a goal."""
        dx = pose.position.x - goal['x']
        dy = pose.position.y - goal['y']
        return math.hypot(dx, dy)
    
    def bresenham_line(self, start_x, start_y, end_x, end_y):
        """
        Casts a virtual ray from start to end on the OccupancyGrid.
        Returns True if the path is clear, False if a wall blocks it.
        Uses Bresenham's Line Algorithm.
        """
        if self.map is None:
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
            if self.map.data[idx] > 0: 
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


    def score_goal(self, goal):
        """Calculate the score for a goal using the greedy strategy.

        score = -alpha * own_distance + beta * (opponent_distance - own_distance)

        Higher score = better goal to pursue.
        - own_distance: how far I am from the goal
        - opponent_distance: how far the opponent is from the goal
        - alpha: weight for my distance
        - beta: weight for competitive advantage
        """

        """Calculate the score for a goal using greedy strategy + Wall Penalties."""
        if self.own_pose is None:
            return -9999.0

        own_dist = self.euclidean_distance(self.own_pose, goal)

        # Check if a wall is in the way 
        if not self.bresenham_line(self.own_pose.position.x, self.own_pose.position.y, goal['x'], goal['y']):
            # if a wall blocks the straight path add a heavy penalty.
            # This forces the robot to finish all goals on its own side before crossing the wall.
            own_dist += WALL_PENALTY

        if self.opponent_pose is not None:
            opp_dist = self.euclidean_distance(self.opponent_pose, goal)
            # Apply the same wall penalty to the opponent's distance for fair competition
            if not self.bresenham_line(self.opponent_pose.position.x, self.opponent_pose.position.y, goal['x'], goal['y']):
                opp_dist += WALL_PENALTY
                
            competitive_advantage = opp_dist - own_dist
        else:
            competitive_advantage = 0.0  

        return -ALPHA * own_dist + BETA * competitive_advantage


        # if self.own_pose is None:
        #     return 0.0

        # own_dist = self.euclidean_distance(self.own_pose, goal)

        # if self.opponent_pose is not None:
        #     opp_dist = self.euclidean_distance(self.opponent_pose, goal)
        #     competitive_advantage = opp_dist - own_dist
        # else:
        #     competitive_advantage = 0.0  # no info about opponent

        # return -ALPHA * own_dist + BETA * competitive_advantage


    

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
                dist = self.euclidean_distance(self.own_pose, g)
                if dist <= new_radius:
                    reachable_goals.append(g)
            
            # If no goals are within radius, expand by 1 meter
            if not reachable_goals:
                new_radius += 2.0
        
        if not reachable_goals:
            return None

        best_goal = max(reachable_goals, key=lambda g: self.score_goal(g))
        return best_goal
        
    def send_goal_to_nav2(self, goal):
        """Send the selected goal to Nav2(request)"""

        if not self.nav2_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Nav2 action server not available')
            return

        # If the robot has a goal
        if self.own_goal_handle is not None:
            self.pending_goal = goal
            cancel_future=self.own_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.on_cancel_done)
        # Send the new goal immediately
        else:
            self.actually_send_goal(goal)



    # Callback for when the cancel request is done, to send the pending goal
    def on_cancel_done(self, future):
        self.own_goal_handle = None
        if self.pending_goal is not None:
            goal_to_send = self.pending_goal
            self.pending_goal = None   
            self.actually_send_goal(goal_to_send)


    # Added these two callbacks for goal response to handle acceptance/rejection
    def goal_result_callback(self, future):
        self.current_goal = None   # reset current goal when result is received (either success or failure), to allow selecting a new goal
        self.navigating = False
        self.goal_handle = None  # PROVA reset the receipt

    # Helper function to actually send the goal to Nav2 after canceling the previous one
    def actually_send_goal(self, goal):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = goal['x']
        goal_msg.pose.pose.position.y = goal['y']
        goal_msg.pose.pose.orientation.w = 1.0

        send_future = self.nav2_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self.goal_response_callback)

        self.current_goal = goal
        self.last_goal_sent_time = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().info(
            f'{self.robot_name} navigating to ({goal["x"]:.2f}, {goal["y"]:.2f})'
        )
    
    # Callback to reset current goal when result is received (either success or failure), to allow selecting a new goal
    def goal_result_callback(self, future):
        # Ignore the result if there is a process to set a new goal (pending goal)
        if self.pending_goal is not None:
            return
        self.own_goal_handle = None
        self.current_goal = None


    def goal_response_callback(self, future):
        """Estabilish if the goal was accepted or rejected by Nav2, 
        if accepted, reset the current goal to None in order to find a new one (done by goal_result_callback)"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected!')
            self.current_goal = None
            self.navigating = False
            self.goal_handle = None #PROVA
            return
        # <-- SAVE THE RECEIPT HERE PROVA
        self.goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)
        
    def is_goal_reached(self):
        """Check if the current goal has been reached."""
        if self.current_goal is None or self.own_pose is None:
            return False
        return self.euclidean_distance(self.own_pose, self.current_goal) <= GOAL_REACHED_THRESHOLD
    

    def should_switch_goal(self, best_goal):
        # Prevent switching goals too frequently by enforcing a minimum time between goal changes
        now = self.get_clock().now().nanoseconds / 1e9
        if self.last_goal_sent_time and (now - self.last_goal_sent_time) < MIN_GOAL_DURATION:
            return False

        best_score = self.score_goal(best_goal)
        current_score = self.score_goal(self.current_goal)
    
        # Adaptive threshold that increases as we get closer to the current goal, to prevent oscillations near the goal
        dist_to_current = self.euclidean_distance(self.own_pose, self.current_goal)
        adaptive_threshold = SWITCH_THRESHOLD + (1.0 / (dist_to_current + 0.1))
        return best_score > current_score + adaptive_threshold
    


    def select_and_send_goal(self):
        
        self.get_logger().info(f'Goals: {len(self.goals)}, Own pose: {self.own_pose is not None}')
        """Main loop: select best goal and send to Nav2."""
        if self.game_over or not self.goals or self.own_pose is None:
            return

        # Wait until cancel is done before sending a new goal
        if self.pending_goal is not None:
            return  

        best_goal = self.select_best_goal()

        if best_goal is None:
            return

        # If current goal reached or no goal set, select a new one
        if self.own_goal_handle is None or self.is_goal_reached():
            self.send_goal_to_nav2(best_goal)
        # If currently navigating but a better goal is available, cancel current goal and select new one
        elif best_goal != self.current_goal:
            if self.should_switch_goal(best_goal):
                self.send_goal_to_nav2(best_goal)



def main(args=None):
    rclpy.init(args=args)
    node = GoalFunction()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()