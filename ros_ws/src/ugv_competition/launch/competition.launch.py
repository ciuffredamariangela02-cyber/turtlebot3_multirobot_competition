# competition.launch.py
# Launches the full competition:
# - Gazebo with the competition world
# - Nav2 localization (AMCL + map_server)
# - Nav2 navigation (planner, controller, etc.)
# - Game Master node
# - Goal Function node for each robot

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Package directories
    pkg_ugv = get_package_share_directory('ugv_competition')
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    # Launch arguments
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_ugv, 'maps', 'custom_map.yaml'),
        description='Path to map file')

    map_file = LaunchConfiguration('map')

    # --- Gazebo (t=0s) ---
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_gazebo, 'launch', 'custom_world.launch.py')
        )
    )

    # --- Localization: AMCL + map_server (t=0s) ---
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': 'true',
            'params_file': os.path.join(pkg_ugv, 'config', 'nav2_params.yaml'),
        }.items()
    )

    # --- Initial pose (t=20s) ---
    initial_pose_cmd = TimerAction(
        period=20.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'topic', 'pub', '--times', '5', '/initialpose',
                    'geometry_msgs/msg/PoseWithCovarianceStamped',
                    '{"header": {"frame_id": "map"}, "pose": {"pose": {"position": {"x": 2.5, "y": -1.5, "z": 0.0}, "orientation": {"w": 1.0}}, "covariance": [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853]}}'],
                output='screen'
            )
        ]
    )

    # --- Navigation: planner, controller, bt_navigator (t=30s) ---
    navigation_cmd = TimerAction(
        period=30.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'true',
                    'params_file': os.path.join(pkg_ugv, 'config', 'nav2_params.yaml'),
                }.items()
            )
        ]
    )

    # --- Game Master (t=45s) ---
    game_master_cmd = TimerAction(
        period=45.0,
        actions=[Node(
            package='ugv_competition',
            executable='game_master',
            name='game_master',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )]
    )

    # --- Goal Function Robot 1 (t=47s) ---
    goal_function_robot1_cmd = TimerAction(
        period=47.0,
        actions=[Node(
            package='ugv_competition',
            executable='goal_function',
            name='goal_function_robot1',
            output='screen',
            parameters=[{
                'robot_name': 'robot1',
                'use_namespace': False,
                'use_sim_time': True
            }]
        )]
    )

    # --- Goal Function Robot 2 (commented out for single robot test) ---
    # goal_function_robot2_cmd = TimerAction(
    #     period=47.0,
    #     actions=[Node(
    #         package='ugv_competition',
    #         executable='goal_function',
    #         name='goal_function_robot2',
    #         output='screen',
    #         parameters=[{
    #             'robot_name': 'robot2',
    #             'use_namespace': True,
    #             'use_sim_time': True
    #         }]
    #     )]
    # )

    return LaunchDescription([
        map_arg,
        gazebo,
        localization,
        initial_pose_cmd,
        navigation_cmd,
        game_master_cmd,
        goal_function_robot1_cmd,
    ])