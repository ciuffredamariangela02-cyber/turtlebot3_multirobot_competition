import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'ugv_competition'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),  # Launch files
        (os.path.join('share', package_name, 'urdf_ws'), glob('urdf_ws/*')), # URDF files for Gazebo
        (os.path.join('share', package_name, 'config'), glob('config/*')), # Config files (e.g., Nav2 params)
        (os.path.join('share', package_name, 'maps'), glob('maps/*')), # Map files
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')), # Gazebo world files
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')), # rviz config
    
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='UGV Competition package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'game_master = ugv_competition.game_master:main',
            'goal_function = ugv_competition.goal_function:main',
            'robot_controller = ugv_competition.robot_controller:main',
            'tf_relay = ugv_competition.tf_relay:main',
            'robot_label = ugv_competition.robot_label:main',
        ],
    },
)
