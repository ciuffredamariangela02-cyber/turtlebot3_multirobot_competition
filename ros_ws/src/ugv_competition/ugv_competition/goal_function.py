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
GOAL_REACHED_THRESHOLD = 0.2  # meters
MAX_GOAL_DISTANCE = 1.0  # max distance to consider a goal reachable
MAX_DISTANCE_ARENA = 10.0  # max distance in the arena for goal selection

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

        pose_qos = QoSProfile(depth=10)
        pose_qos.reliability = ReliabilityPolicy.RELIABLE
        pose_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.create_subscription(
            PoseWithCovarianceStamped,
            own_pose_topic,
            self.own_pose_callback,
            pose_qos)

        # Subscribe to opponent pose
        self.create_subscription(
            PoseWithCovarianceStamped,
            opponent_pose_topic,
            self.opponent_pose_callback,
            pose_qos)

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
        self.goal_handle = None  # PROVA

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

    def euclidean_distance(self, pose, goal):
        """Calculate Euclidean distance between a pose and a goal."""
        dx = pose.position.x - goal['x']
        dy = pose.position.y - goal['y']
        return math.sqrt(dx**2 + dy**2)
    
    #METODO HARDCODED PROVA
    def calculate_path_distance(self, pose, goal):
        """
        Calcola la distanza di navigazione REALE, aggirando il muro centrale.
        Il muro si trova a y = -1.0 e blocca la zona da x = -1.0 a x = 3.0.
        Il varco libero è a sinistra (x < -1.0).
        """
        robot_x = pose.position.x
        robot_y = pose.position.y
        goal_x = goal['x']
        goal_y = goal['y']
        
        #Calcolo la distanza standard in linea d'aria
        straight_dist = math.sqrt((robot_x - goal_x)**2 + (robot_y - goal_y)**2)

        # Controllo se robot e goal sono su lati opposti del muro (y = -1.0)
        opposite_sides = (robot_y > -1.0 and goal_y < -1.0) or (robot_y < -1.0 and goal_y > -1.0)
        
        if opposite_sides:
            # Se ENTRAMBI sono già nella zona del varco (a sinistra di x = -1.0), la linea d'aria va bene
            if robot_x < -1.0 and goal_x < -1.0:
                return straight_dist
            else:
                # IL ROBOT DEVE AGGIRARE IL MURO
                # distanza dal robot al centro del varco (x=-2.0, y=-1.0)
                # e poi dal varco al goal.
                gap_x = -2.0
                gap_y = -1.0
                dist_to_gap = math.sqrt((robot_x - gap_x)**2 + (robot_y - gap_y)**2)
                dist_from_gap_to_goal = math.sqrt((goal_x - gap_x)**2 + (goal_y - gap_y)**2)
                
                # Aggiungere un "Malus" (es. +1.5 metri) per disincentivare i cambi di zona.
                # Il robot preferirà pulire prima tutta la sua metà campo prima di passare all'altra!
                return dist_to_gap + dist_from_gap_to_goal + 1.5 

        # Se sono nello stesso lato del campo, la linea d'aria va benissimo
        return straight_dist

    def score_goal(self, goal):
        """Calculate the score for a goal using the greedy strategy.

        score = -alpha * own_distance + beta * (opponent_distance - own_distance)

        Higher score = better goal to pursue.
        - own_distance: how far I am from the goal
        - opponent_distance: how far the opponent is from the goal
        - alpha: weight for my distance
        - beta: weight for competitive advantage
        """

        #Calcola il punteggio del goal con logica competitiva egoista pura.
    
        if self.own_pose is None:
         return -9999.0

        own_dist = self.calculate_path_distance(self.own_pose, goal)

        # 1. Se non sappiamo dov'è l'avversario, valutiamo solo in base alla nostra distanza.
        # Ritorno un numero negativo (più è vicino allo 0, meglio è).
        if self.opponent_pose is None:
            score = -own_dist
            
            # Isteresi (Bonus di Fedeltà)
            if (self.current_goal is not None and 
                abs(goal['x'] - self.current_goal['x']) < 1e-6 and 
                abs(goal['y'] - self.current_goal['y']) < 1e-6):
                score += 0.5
            return score

        # 2. Conosciamo la posizione dell'avversario.
        opp_dist = self.calculate_path_distance(self.opponent_pose, goal)

        # -------------------------------------------------------------
        # REGOLA COMPETITIVA (CUT-OFF): 
        # Se l'avversario è più vicino a questo goal di un certo margine 
        # (es. ha 40 cm di vantaggio), NON CI PROVARE NEMMENO. Perderesti.
        # Assegniamo un punteggio bassissimo per scartarlo all'istante.
        # -------------------------------------------------------------
        margin_of_defeat = 0.4 # metri di vantaggio dell'avversario
        
        if opp_dist < (own_dist - margin_of_defeat):
            return -9999.0  # Scartato.

        # -------------------------------------------------------------
        # SE SIAMO IN GARA (o se siamo in vantaggio noi):
        # Il punteggio è primariamente dettato dalla nostra distanza.
        # Ma aggiungiamo un piccolissimo "BETA" per infastidire l'avversario 
        # SOLO nei casi in cui le distanze sono simili (contesa).
        # -------------------------------------------------------------
        score = -own_dist
        
        # Se siamo in contesa (distanze simili), diamo un piccolo bonus 
        # per rubarglielo, ma NON COSÌ TANTO da farci viaggiare dall'altra
        # parte della mappa ignorando i goal vicini e sicuri.
        if abs(own_dist - opp_dist) < 1.0: 
            competitive_bonus = (opp_dist - own_dist) * BETA  # Usa il tuo parametro BETA (0.5)
            score += competitive_bonus

        # Isteresi (Bonus di Fedeltà)
        if (self.current_goal is not None and 
            abs(goal['x'] - self.current_goal['x']) < 1e-6 and 
            abs(goal['y'] - self.current_goal['y']) < 1e-6):
            score += 0.5

        return score

    #    if self.own_pose is None:
    #         return 0.0

    #     own_dist = self.calculate_path_distance(self.own_pose, goal)

    #     if self.opponent_pose is not None:
    #         opp_dist = self.calculate_path_distance(self.opponent_pose, goal)
    #         competitive_advantage = opp_dist - own_dist
    #     else:
    #         competitive_advantage = 0.0  # no info about opponent

    #     score = -ALPHA * own_dist + BETA * competitive_advantage
        
    #     if (self.current_goal is not None 
    #             and abs(goal['x'] - self.current_goal['x']) < 1e-6 
    #             and abs(goal['y'] - self.current_goal['y']) < 1e-6):
    #         score += 0.5

    #     return score
       
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
                dist = self.calculate_path_distance(self.own_pose, g)
                if dist <= new_radius:
                    reachable_goals.append(g)
            
            # If no goals are within radius, expand by 1 meter
            if not reachable_goals:
                new_radius += 1.0
        
        if not reachable_goals:
            return None

        best_goal = max(reachable_goals, key=lambda g: self.score_goal(g))
        return best_goal
        
    def send_goal_to_nav2(self, goal):
        """Send the selected goal to Nav2."""
        if not self.nav2_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Nav2 action server not available')
            return

        # Added, due to the fact that the goal function must be computed continuosly,
        # so that the robot can change goal, we need to cancel the current goal before sending a new one, 
        # otherwise Nav2 will reject the new goal

       # if self.navigating:
       #     self.nav2_client.cancel_all_goals_async()

       # PROVA
       #cancel the current driving task in ROS 2
        if self.navigating and self.goal_handle is not None:
            self.get_logger().info('Canceling previous navigation task to pursue a better one.')
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = goal['x']
        goal_msg.pose.pose.position.y = goal['y']
        goal_msg.pose.pose.orientation.w = 1.0

        send_goal_future = self.nav2_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
        self.current_goal = goal
        self.navigating = True
        self.get_logger().info(
            f'{self.robot_name} navigating to goal ({goal["x"]:.2f}, {goal["y"]:.2f})')

    # Added these two callbacks for goal response to handle acceptance/rejection
    def goal_result_callback(self, future):
        self.current_goal = None   # reset current goal when result is received (either success or failure), to allow selecting a new goal
        self.navigating = False
        self.goal_handle = None  # PROVA reset the receipt
    
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
        return self.calculate_path_distance(self.own_pose, self.current_goal) < GOAL_REACHED_THRESHOLD

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
            
        else:
        # Goal in corso e non ancora raggiunto:  
        # rimanda SOLO se è cambiata la scelta migliore
         best_goal = self.select_best_goal()
        if best_goal and (
            abs(best_goal['x'] - self.current_goal['x']) > 1e-6 or
            abs(best_goal['y'] - self.current_goal['y']) > 1e-6
        ):
            self.send_goal_to_nav2(best_goal)


def main(args=None):
    rclpy.init(args=args)
    node = GoalFunction()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
