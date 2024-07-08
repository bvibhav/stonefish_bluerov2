# from launch_ros.actions import Node
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
import os

from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python import get_package_share_directory

def generate_launch_description():
    # launch_param = LaunchConfiguration('launch_param')
    
    # include another launch file
    launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('stonefish_ros2'), 
                'launch/stonefish_simulator.launch.py')),
        launch_arguments={
            'simulation_data': os.path.join(
                get_package_share_directory('stonefish_bluerov2'), 
                'data/'),
            'scenario_desc': os.path.join(
                get_package_share_directory('stonefish_bluerov2'), 
                'scenarios/bluerov2_tank.scn'),
            'simulation_rate': '100.0',
            'window_res_x': '960',
            'window_res_y': '1056',
            'rendering_quality': 'high',
        }.items()
    )

    ardusim_patch = Node(
            package='stonefish_bluerov2',
            namespace='bluerov2',
            executable='ardusim_patch.py',
            name='ardusim_patch',
            output='screen',
            emulate_tty='true',
        )

    return LaunchDescription([
        launch_include,
        ardusim_patch, 
    ])