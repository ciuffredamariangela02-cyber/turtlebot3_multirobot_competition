#launch file to choose a specific metric for each robot

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():

    pkg_ugv = get_package_share_directory('ugv_competition')
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    # --- Launch Arguments ---
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_ugv, 'maps', 'custom_map.yaml'),
        description='Path to map file')

    # Metric arguments for each robot
    robot1_metric_arg = DeclareLaunchArgument(
        'robot1_metric',
        default_value='euclidean',
        description='Metric for robot1 (options: euclidean, manhattan, estimated_time, cluster)')

    robot2_metric_arg = DeclareLaunchArgument(
        'robot2_metric',
        default_value='manhattan',
        description='Metric for robot2 (options: euclidean, manhattan, estimated_time, cluster)')

    map_file = LaunchConfiguration('map')

    # Gazebo (t=0s)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_gazebo, 'launch', 'custom_world.launch.py')
        )
    )

    # TF Relay (t=0s)
    tf_relay = Node(
        package='ugv_competition',
        executable='tf_relay',
        name='tf_relay',
        output='screen',
        parameters=[{
            'robot_namespaces': ['robot1', 'robot2'],
            'use_sim_time': True
        }]
    )

    # Rviz (t=7s)
    rviz_cmd = TimerAction(
        period=7.0,
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_ugv, 'rviz', 'competition.rviz')],
            output='screen'
        )]
    )

    nav2_robot1 = TimerAction(
        period=10.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ugv, 'launch', 'navigation_robot.launch.py')
            ),
            launch_arguments={
                'namespace': 'robot1',
                'map': map_file,
                'use_sim_time': 'true',
                'params_file': os.path.join(pkg_ugv, 'config', 'nav2_robot1.yaml'),
            }.items()
        )]
    )

    nav2_robot2 = TimerAction(
        period=10.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ugv, 'launch', 'navigation_robot.launch.py')
            ),
            launch_arguments={
                'namespace': 'robot2',
                'map': map_file,
                'use_sim_time': 'true',
                'params_file': os.path.join(pkg_ugv, 'config', 'nav2_robot2.yaml'),
            }.items()
        )]
    )

    # --- Initial pose Robot 1  ---
    initial_pose_robot1_cmd = TimerAction(
        period=25.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'topic', 'pub', '--times', '10', '/robot1/initialpose',
                    'geometry_msgs/msg/PoseWithCovarianceStamped',
                    '{"header": {"frame_id": "map"}, "pose": {"pose": {"position": {"x": 2.5, "y": -1.5, "z": 0.0}, "orientation": {"w": 1.0}}, "covariance": [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853]}}'],
                output='screen'
            )
        ]
    )

    # --- Initial pose Robot 2 ---
    initial_pose_robot2_cmd = TimerAction(
        period=25.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'topic', 'pub', '--times', '10', '/robot2/initialpose',
                    'geometry_msgs/msg/PoseWithCovarianceStamped',
                    '{"header": {"frame_id": "map"}, "pose": {"pose": {"position": {"x": 2.5, "y": -2.5, "z": 0.0}, "orientation": {"w": 1.0}}, "covariance": [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853]}}'],
                output='screen'
            )
        ]
    )

    # --- Game Master ---
    game_master_cmd = TimerAction(
        period=40.0,
        actions=[Node(
            package='ugv_competition',
            executable='game_master',
            name='game_master',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )]
    )

    # --- Goal Function Robot 1 ---
    # Passing the metric argument to the node parameters
    goal_function_robot1_cmd = TimerAction(
        period=45.0,
        actions=[Node(
            package='ugv_competition',
            executable='goal_function',
            name='goal_function_robot1',
            output='screen',
            parameters=[{
                'robot_name': 'robot1',
                'metric_name': LaunchConfiguration('robot1_metric'), # <--- NEW
                'use_namespace': True,
                'use_sim_time': True
            }]
        )]
    )

    # --- Goal Function Robot 2 ---
    # Passing the metric argument to the node parameters
    goal_function_robot2_cmd = TimerAction(
        period=45.0,
        actions=[Node(
            package='ugv_competition',
            executable='goal_function',
            name='goal_function_robot2',
            output='screen',
            parameters=[{
                'robot_name': 'robot2',
                'metric_name': LaunchConfiguration('robot2_metric'), # <--- NEW
                'use_namespace': True,
                'use_sim_time': True
            }]
        )]
    )

    # Robot Label 
    robot_label_cmd = TimerAction(
        period=41.0,
        actions=[Node(
            package='ugv_competition',
            executable='robot_label',
            name='robot_label',
            output='screen',
            parameters=[{'use_sim_time':True}]
        )]
    )

    return LaunchDescription([
        map_arg,
        robot1_metric_arg,  
        robot2_metric_arg, 
        gazebo,
        tf_relay,
        rviz_cmd,
        nav2_robot1,
        nav2_robot2,
        initial_pose_robot1_cmd,
        initial_pose_robot2_cmd,
        game_master_cmd,
        goal_function_robot1_cmd,
        goal_function_robot2_cmd,
        robot_label_cmd
    ])
