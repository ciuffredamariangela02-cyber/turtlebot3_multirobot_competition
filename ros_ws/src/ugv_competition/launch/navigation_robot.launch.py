# navigation_robot.launch.py
# This launch file starts Nav2 for a single robot with the correct namespace.
# It uses RewrittenYaml to rewrite the robot-specific nav2_robotX.yaml adding the
# robot namespace as root key, so all Nav2 nodes correctly read their parameters.
# Two separate YAML files are used (nav2_robot1.yaml and nav2_robot2.yaml) to
# explicitly specify namespace-prefixed frame IDs and topic names for each robot.
#
# TF remappings ensure Nav2 publishes on the global /tf topic instead of the
# namespaced /robot1/tf, which is handled by the tf_relay node.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart = LaunchConfiguration('autostart', default='true')
    params_file = LaunchConfiguration('params_file')
    map_file = LaunchConfiguration('map')

    lifecycle_nodes_localization = [
        'map_server',
        'amcl',
    ]

    lifecycle_nodes_navigation = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
        'collision_monitor',
    ]

    # TF remappings: Nav2 nodes publish TF on relative 'tf' topic.
    # With PushRosNamespace this becomes /robot1/tf but we need global /tf.
    remappings = [('tf', '/tf'), ('tf_static', '/tf_static')]

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites={},
        convert_types=True
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='robot1',
        description='Robot namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically start Nav2')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            get_package_share_directory('ugv_competition'),
            'config', 'nav2_params.yaml'),
        description='Nav2 parameters file')

    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            get_package_share_directory('ugv_competition'),
            'maps', 'custom_map.yaml'),
        description='Map file')

    load_nodes = GroupAction([
        PushRosNamespace(namespace),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[configured_params, {'yaml_filename': map_file}, {'use_sim_time': True}],
            remappings=remappings),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[
                configured_params,
                {'scan_topic': 'scan'},
                {'transform_tolerance': 5.0},
                {'use_sim_time': True}, 
                {'base_frame_id': [namespace, '/base_footprint']},
                {'odom_frame_id': [namespace, '/odom']},           
                {'global_frame_id': 'map'},  
            ],
            remappings=remappings),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[configured_params, {'use_sim_time': True}],
            remappings=remappings + [('scan', [namespace, '/scan'])]),

        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            parameters=[configured_params, {'use_sim_time': True}],
            remappings=remappings),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[configured_params, {'use_sim_time': True}],
            remappings=remappings + [('scan', [namespace, '/scan'])]),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[configured_params, {'use_sim_time': True}],
            remappings=remappings),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[configured_params, {'use_sim_time': True}],
            remappings=remappings),

        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[configured_params, {'use_sim_time': True}],
            remappings=remappings),

        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[configured_params, {'use_sim_time': True}],
            remappings=remappings),

        Node(
            package='nav2_collision_monitor',
            executable='collision_monitor',
            name='collision_monitor',
            output='screen',
            parameters=[configured_params, {'use_sim_time': True}],
            remappings=remappings + [('scan', [namespace, '/scan'])]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'autostart': autostart,
                'node_names': lifecycle_nodes_localization,  
                'use_sim_time': use_sim_time,
                'bond_timeout': 0.0,
            }]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'autostart': autostart,
                'node_names': lifecycle_nodes_navigation,    
                'use_sim_time': use_sim_time,
                'bond_timeout': 0.0,
                'attempt_respawn_reconnection': True, # to retry connecting to the lifecycle manager if it is not available
            }]),
    ])

    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_map_cmd)
    ld.add_action(load_nodes)

    return ld