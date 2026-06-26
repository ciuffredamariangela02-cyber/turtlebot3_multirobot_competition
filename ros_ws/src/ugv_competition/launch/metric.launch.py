# metric.launch.py
# Launch file for the competition with configurable metrics, map and goal placement.
#
# Example:
#   ros2 launch ugv_competition metric.launch.py robot1_metric:=euclidean robot2_metric:=manhattan map_name:=custom
#   ros2 launch ugv_competition metric.launch.py robot1_metric:=euclidean robot2_metric:=manhattan map_name:=symmetric
#   ros2 launch ugv_competition metric.launch.py map_name:=symmetric goal_seed:=42 goal_placement:=random
#   ros2 launch ugv_competition metric.launch.py robot1_metric:=estimated_time_to_goal robot2_metric:=estimated_time_to_goal map_name:=symmetric goal_seed:=42 goal_placement:=random   
#
# Set goal seed to 42 for reproducibility with the reported simulations in the report
# Available metrics: euclidean, manhattan, estimated_time
# Available maps: custom, symmetric
# Available goal placements: random, symmetric (default: random for custom, symmetric for symmetric)

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):

    pkg_ugv = get_package_share_directory('ugv_competition')
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    # Map selected
    map_name = context.launch_configurations.get('map_name', 'custom')

    # Goal placement - if no goal placement has been specified, symmetric is used for symmetric map, and random for the others
    goal_placement = context.launch_configurations.get('goal_placement', '')
    if not goal_placement:
        goal_placement = 'symmetric' if map_name == 'symmetric' else 'random'

    # Metric
    robot1_metric = context.launch_configurations.get('robot1_metric', 'euclidean')
    robot2_metric = context.launch_configurations.get('robot2_metric', 'euclidean')

    # Select world launch file, map file and spawn positions based on map_name
    if map_name == 'symmetric':
        world_launch = os.path.join(pkg_tb3_gazebo, 'launch', 'simmetric_world.launch.py')
        map_file = os.path.join(pkg_ugv, 'maps', 'simmetric_map.yaml')
        robot1_x, robot1_y, robot1_yaw = '0.0', '-0.3', '0.0'
        robot2_x, robot2_y, robot2_yaw = '0.0', '0.3', '3.14159'
        robot1_oz, robot1_ow = '0.0', '1.0'
        robot2_oz, robot2_ow = '1.0', '0.0'
    else:
        world_launch = os.path.join(pkg_tb3_gazebo, 'launch', 'custom_world.launch.py')
        map_file = os.path.join(pkg_ugv, 'maps', 'custom_map.yaml')
        robot1_x, robot1_y, robot1_yaw = '2.5', '-1.5', '0.0'
        robot2_x, robot2_y, robot2_yaw = '2.5', '-2.5', '0.0'
        robot1_oz, robot1_ow = '0.0', '1.0'
        robot2_oz, robot2_ow = '0.0', '1.0'

    # Gazebo (t=0s)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(world_launch)
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

    # RViz (t=7s)
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

    # Nav2 Robot 1 (t=10s)
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

    # Nav2 Robot 2 (t=10s)
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

    # Initial pose Robot 1 (t=25s)
    initial_pose_robot1_cmd = TimerAction(
        period=25.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'topic', 'pub', '--times', '15', '/robot1/initialpose',
                    'geometry_msgs/msg/PoseWithCovarianceStamped',
                    f'{{"header": {{"frame_id": "map"}}, "pose": {{"pose": {{"position": {{"x": {robot1_x}, "y": {robot1_y}, "z": 0.0}}, "orientation": {{"x": 0.0, "y": 0.0, "z": {robot1_oz}, "w": {robot1_ow}}}}}, "covariance": [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853]}}}}'],
                output='screen'
            )
        ]
    )

    # Initial pose Robot 2 (t=25s)
    initial_pose_robot2_cmd = TimerAction(
        period=25.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'topic', 'pub', '--times', '15', '/robot2/initialpose',
                    'geometry_msgs/msg/PoseWithCovarianceStamped',
                    f'{{"header": {{"frame_id": "map"}}, "pose": {{"pose": {{"position": {{"x": {robot2_x}, "y": {robot2_y}, "z": 0.0}}, "orientation": {{"x": 0.0, "y": 0.0, "z": {robot2_oz}, "w": {robot2_ow}}}}}, "covariance": [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853]}}}}'],
                output='screen'
            )
        ]
    )

    # Game Master (t=40s)
    game_master_cmd = TimerAction(
        period=40.0,
        actions=[Node(
            package='ugv_competition',
            executable='game_master',
            name='game_master',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'goal_placement': goal_placement,
                'goal_seed': int(context.launch_configurations.get('goal_seed', '-1'))
            }]
        )]
    )

    # Goal Function Robot 1 (t=45s)
    goal_function_robot1_cmd = TimerAction(
        period=45.0,
        actions=[Node(
            package='ugv_competition',
            executable='goal_function',
            name='goal_function_robot1',
            output='screen',
            parameters=[{
                'robot_name': 'robot1',
                'metric_name': robot1_metric,
                'use_namespace': True,
                'use_sim_time': True
            }]
        )]
    )

    # Goal Function Robot 2 (t=45s)
    goal_function_robot2_cmd = TimerAction(
        period=45.0,
        actions=[Node(
            package='ugv_competition',
            executable='goal_function',
            name='goal_function_robot2',
            output='screen',
            parameters=[{
                'robot_name': 'robot2',
                'metric_name': robot2_metric,
                'use_namespace': True,
                'use_sim_time': True
            }]
        )]
    )

    # Robot Label (t=41s)
    robot_label_cmd = TimerAction(
        period=41.0,
        actions=[Node(
            package='ugv_competition',
            executable='robot_label',
            name='robot_label',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )]
    )

    return [
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
        robot_label_cmd,
    ]


def generate_launch_description():

    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='custom',
        description='Map to use: custom or symmetric')

    robot1_metric_arg = DeclareLaunchArgument(
        'robot1_metric',
        default_value='euclidean',
        description='Metric for robot1: euclidean, manhattan, estimated_time, cluster')

    robot2_metric_arg = DeclareLaunchArgument(
        'robot2_metric',
        default_value='euclidean',
        description='Metric for robot2: euclidean, manhattan, estimated_time, cluster')

    goal_seed_arg = DeclareLaunchArgument(
        'goal_seed',
        default_value='-1',
        description='Random seed for goal generation. -1 = random seed.')

    return LaunchDescription([
        map_name_arg,
        robot1_metric_arg,
        robot2_metric_arg,
        goal_seed_arg,
        OpaqueFunction(function=launch_setup),
    ])
