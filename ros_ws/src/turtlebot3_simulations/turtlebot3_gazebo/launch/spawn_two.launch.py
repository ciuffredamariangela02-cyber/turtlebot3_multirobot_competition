# spawn_two.launch.py
# This launch file spawns a single TurtleBot3 robot in Gazebo with a given namespace.
# It is called twice from custom_world.launch.py to spawn robot1 and robot2.
#
# Key approach (adapted from arshadlab/tb3_multi_robot):
# The SDF model file is patched at runtime to inject the robot namespace directly 
# into all Gazebo topic names. This means Gazebo publishes directly on /robot1/tf, 
# /robot1/scan, /robot1/odom etc. with the correct frame IDs (robot1/odom, 
# robot1/base_footprint).
# The ros_gz_bridge uses a namespaced bridge YAML to map these topics to ROS2.
# The robot_state_publisher uses frame_prefix to publish robot1/base_link etc.
#
# A tf_relay node (launched separately in competition.launch.py) subscribes to 
# each robot's namespaced /tf topic and republishes on the global /tf, ensuring 
# all transforms are available in a single TF tree for Nav2 and AMCL.
#
# Note: the /clock topic is bridged only once (by robot1) to avoid duplicate
# publishers that cause clock instability and "jump back in time" errors.

import os
import tempfile
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

sys.path.insert(0, os.path.join(get_package_share_directory('ugv_competition'), 'launch'))
from robot_utils import load_sdf_with_namespace, create_namespaced_bridge_yaml


def generate_launch_description():

    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    model_folder = 'turtlebot3_' + TURTLEBOT3_MODEL

    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')

    model_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models', model_folder, 'model.sdf'
    )

    base_bridge_yaml = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'params', model_folder + '_bridge.yaml'
    )

    # Declare 4 arguments
    x_pose          = LaunchConfiguration('x_pose',          default='0.0')
    y_pose          = LaunchConfiguration('y_pose',          default='0.0')
    robot_name      = LaunchConfiguration('robot_name',      default=TURTLEBOT3_MODEL)
    robot_namespace = LaunchConfiguration('robot_namespace', default='')

    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='X position of the robot')

    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Y position of the robot')

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name', default_value=TURTLEBOT3_MODEL,
        description='Name of the robot in Gazebo')

    declare_robot_namespace_cmd = DeclareLaunchArgument(
        'robot_namespace', default_value='',
        description='ROS namespace of the robot')

    def spawn_robot(context):
        ns   = context.launch_configurations['robot_namespace']
        name = context.launch_configurations['robot_name']
        x    = context.launch_configurations['x_pose']
        y    = context.launch_configurations['y_pose']

        # Patch SDF with namespace — Gazebo will publish on /robot1/tf, /robot1/scan etc.
        sdf_text = load_sdf_with_namespace(model_path, ns)
        tmp_sdf = tempfile.NamedTemporaryFile(suffix='.sdf', delete=False, mode='w')
        tmp_sdf.write(sdf_text)
        tmp_sdf.close()

        # Create namespaced bridge YAML.
        # Only robot1 bridges /clock — bridging it twice causes clock instability.
        bridge_yaml = create_namespaced_bridge_yaml(
            base_bridge_yaml,
            ns,
            include_clock=(ns == 'robot1')
        )

        # Spawn robot in Gazebo using patched SDF
        spawner = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', name,
                '-file', tmp_sdf.name,
                '-x', x,
                '-y', y,
                '-z', '0.01'
            ],
            output='screen',
        )

        # Bridge with namespaced YAML — maps /robot1/scan etc. to ROS2
        bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '--ros-args',
                '-p',
                f'config_file:={bridge_yaml}',
            ],
            parameters=[{'use_sim_time': True}],
            output='screen',
        )

        # Robot state publisher with frame_prefix
        rsp = GroupAction([
            PushRosNamespace(ns),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'true',
                    'frame_prefix': ns
                }.items()
            )
        ])

        return [spawner, bridge, rsp]

    ld = LaunchDescription()
    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_robot_namespace_cmd)
    ld.add_action(OpaqueFunction(function=spawn_robot))

    return ld