import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import TimerAction

def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Symmetric spawn positions relative to arena center 
    x_pose_r1 = LaunchConfiguration('x_pose',    default='0')
    y_pose_r1 = LaunchConfiguration('y_pose',     default='-3.7')
    x_pose_r2 = LaunchConfiguration('x_pose_r2', default='0')
    y_pose_r2 = LaunchConfiguration('y_pose_r2', default='3.7')

    world = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'simmetric_world.world'
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v8 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v8 ', 'on_exit_shutdown': 'true'}.items()
    )

    spawn_turtlebot1_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_two.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose_r1,
            'y_pose': y_pose_r1,
            'robot_name': 'robot1',
            'robot_namespace': 'robot1'
        }.items()
    )

    spawn_turtlebot2_cmd = TimerAction(
        period=5.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_file_dir, 'spawn_two.launch.py')
            ),
            launch_arguments={
                'x_pose': x_pose_r2,
                'y_pose': y_pose_r2,
                'robot_name': 'robot2',
                'robot_namespace': 'robot2'
            }.items()
        )]
    )

    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(
            get_package_share_directory('turtlebot3_gazebo'),
            'models'))

    ld = LaunchDescription()
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(spawn_turtlebot1_cmd)
    ld.add_action(spawn_turtlebot2_cmd)
    ld.add_action(set_env_vars_resources)
    return ld
