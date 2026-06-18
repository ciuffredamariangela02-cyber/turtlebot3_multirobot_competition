import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, PoseStamped
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from tf_transformations import euler_from_quaternion
import math
    
    # Alternative metric to estimate distance from goal
    def manhattan_distance(self,pose,goal):
        '''Calculate Manhattan distance between a pose and a goal'''
        dx = abs(pose.position.x - goal['x'])
        dy = abs(pose.position.y - goal['y'])
        return dx+dy

    # Alternative metric to estimate distance from goal
    def estimated_time_to_goal(self,pose,goal):
        dx = pose.position.x - goal['x']
        dy = pose.position.y - goal['y']
        dist= math.hypot(dx, dy)
        
        orientation=pose.orientation
        q = [orientation.x, orientation.y, orientation.z, orientation.w]
        _,_,yaw=euler_from_quaternion(q)
        angle_to_goal=math.atan2(dy,dx)
        angle_diff=abs(math.atan2(math.sin(angle_to_goal-yaw),math.cos(angle_to_goal-yaw)))

        VX_MAX=0.22
        WZ_MAX=0.8
        
        time_to_rotate = angle_diff/WZ_MAX
        time_to_translate = dist/VX_MAX
        return time_to_rotate+time_to_translate #estimated seconds

    # alternative metric for score_goal method
    def cluster_score(self, goal, radius=2.0):
        count = 0
        for other in self.goals:
            if other == goal:
                continue
            d = math.hypot(goal['x'] - other['x'], goal['y'] - other['y'])
            if d < radius:
                count += 1
        return count 

    def euclidean_distance(self, pose, goal):
        """Calculate Euclidean distance between a pose and a goal."""
        dx = pose.position.x - goal['x']
        dy = pose.position.y - goal['y']
        return math.hypot(dx, dy)
    
