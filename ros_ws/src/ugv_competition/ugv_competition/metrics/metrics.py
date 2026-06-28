# Alternative metric to estimate distance from goal

import math
from tf_transformations import euler_from_quaternion
import math
    

# Constants for estimated time calculation
VX_MAX = 0.2
WZ_MAX = 1.0

def manhattan_distance(pose,goal):
        '''Calculate Manhattan distance between a pose and a goal'''
        dx = abs(goal['x'] - pose.position.x)
        dy = abs(goal['y'] - pose.position.y)
        
        return dx + dy



def estimated_time_to_goal(pose,goal):
        dx = goal['x'] - pose.position.x
        dy = goal['y'] - pose.position.y
        dist = math.hypot(dx, dy)
        
        orientation=pose.orientation
        q = [orientation.x, orientation.y, orientation.z, orientation.w]
        _,_,yaw=euler_from_quaternion(q)
        angle_to_goal=math.atan2(dy,dx)
        angle_diff=abs(math.atan2(math.sin(angle_to_goal-yaw),math.cos(angle_to_goal-yaw)))

        time_to_rotate = angle_diff/WZ_MAX
        time_to_translate = dist/VX_MAX
        estimated_seconds = time_to_rotate + time_to_translate
        return estimated_seconds

# alternative metric for score_goal method
def cluster_score( goal, all_goal, radius=2.0):
        """Count how many other goals are within a certain radius of the target goal."""
        count = 0
        for other in all_goal:
            if other == goal:
                continue
            d = math.hypot(goal['x'] - other['x'], goal['y'] - other['y'])
            if d < radius:
                count += 1
        return count 


def euclidean_distance( pose, goal):
        """Calculate Euclidean distance between a pose and a goal."""
        dx = goal['x'] - pose.position.x
        dy = goal['y'] - pose.position.y
        return math.hypot(dx, dy)
    

