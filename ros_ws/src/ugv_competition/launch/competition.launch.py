# competition.launch.py
# Launches the full competition:
# - Gazebo with the competition world
# - Two TurtleBot3 robots with separate namespaces
# - Nav2 for each robot
# - Game Master node
# - Goal Function node for each robot
 
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
 
 
def generate_launch_description():
 
    # Package directories
    pkg_ugv = get_package_share_directory('ugv_competition')
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_nav2 = get_package_share_directory('nav2_bringup')
 
    # Launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_ugv, 'worlds', 'competition.world'),
        description='Path to Gazebo world file')
 
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_ugv, 'maps', 'arena_map.yaml'),
        description='Path to map file')
 
    world = LaunchConfiguration('world')
    map_file = LaunchConfiguration('map')
 
    # --- Gazebo ---
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_gazebo, 'launch', 'turtlebot3_world.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )
 
    # --- Robot 1 (namespace: robot1) ---
    robot1 = GroupAction([
        PushRosNamespace('robot1'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': map_file,
                'use_sim_time': 'true',
                'params_file': os.path.join(pkg_ugv, 'config', 'nav2_params.yaml'),
                'namespace': 'robot1',
            }.items()
        )
    ])
 
    # --- Robot 2 (namespace: robot2) ---
    robot2 = GroupAction([
        PushRosNamespace('robot2'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': map_file,
                'use_sim_time': 'true',
                'params_file': os.path.join(pkg_ugv, 'config', 'nav2_params.yaml'),
                'namespace': 'robot2',
            }.items()
        )
    ])
 
    # --- Game Master ---
    game_master = Node(
        package='ugv_competition',
        executable='game_master',
        name='game_master',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
 
    # --- Goal Function Robot 1 ---
    goal_function_robot1 = Node(
        package='ugv_competition',
        executable='goal_function',
        name='goal_function_robot1',
        output='screen',
        parameters=[{
            'robot_name': 'robot1',
            'use_sim_time': True
        }]
    )
 
    # --- Goal Function Robot 2 ---
    goal_function_robot2 = Node(
        package='ugv_competition',
        executable='goal_function',
        name='goal_function_robot2',
        output='screen',
        parameters=[{
            'robot_name': 'robot2',
            'use_sim_time': True
        }]
    )
 
    return LaunchDescription([
        world_arg,
        map_arg,
        gazebo,
        robot1,
        robot2,
        game_master,
        goal_function_robot1,
        goal_function_robot2,
    ])