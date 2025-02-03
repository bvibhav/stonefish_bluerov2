from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python import get_package_share_directory

def generate_launch_description():
    # include another launch file
    launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(get_package_share_directory('stonefish_ros2') + \
                                      '/launch/stonefish_simulator.launch.py'),
        launch_arguments={
            'simulation_data': get_package_share_directory('stonefish_bluerov2')+'/data/',
            # 'scenario_desc': get_package_share_directory('stonefish_bluerov2')+'/scenarios/bluesim.scn',
            'scenario_desc': get_package_share_directory('stonefish_bluerov2')+'/scenarios/bluerov2_turbine.scn',
            'simulation_rate': '100.0',
            'window_res_x': '1080',
            'window_res_y': '936',
            'rendering_quality': 'high',
        }.items(),
    )

    tf_ned_enu = Node(
        package = "tf2_ros", 
        executable = "static_transform_publisher",
        arguments = ["0", "0", "0", "1.571", "0", "3.142", "world_ned", "world"]
    )

    tf_odom_ned = Node(
        package='bluerov2_interface',
        # namespace=namespace_val,
        executable='world_tf.py',
        name='odom_ned',
    )

    tf_base_cam_left = Node(
        package = "tf2_ros", 
        executable = "static_transform_publisher",
        arguments = ["0.16", "-0.0725", "0.15", "1.571", "0", "1.571", "base_link", "bluerov2/camera_left"]
    )

    tf_base_cam_right = Node(
        package = "tf2_ros", 
        executable = "static_transform_publisher",
        arguments = ["0.16", "0.0725", "0.15", "1.571", "0", "1.571", "base_link", "bluerov2/camera_right"]
    )

    tf_multibeam = Node(
        package = "tf2_ros", 
        executable = "static_transform_publisher",
        arguments = ["0.2", "0.0", "0.2", "0", "0", "0", "base_link", "bluerov2/multibeam"]
    )

    rviz_node = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d',
            get_package_share_directory('bluerov2_interface') + \
            '/config/bluerov2.rviz']
    )

    robot_description = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='bluerov2',
            output='screen',
            parameters=[{'robot_description': '/home/bvibhav/ros_hume/src/stonefish_bluerov2/data/urdf/bluerov2.urdf'}],
        )

    ardusim_bluerov2 = Node(
            package='stonefish_bluerov2',
            namespace='bluerov2',
            executable='ardusim_patch.py',
            name='ardusim_bluerov2',
            output='screen',
            emulate_tty='true',
        )

    ardusim_blueboat = Node(
            package='stonefish_bluerov2',
            namespace='blueboat',
            executable='ardusim_patch.py',
            name='ardusim_blueboat',
            output='screen',
            emulate_tty='true',
        )

    return LaunchDescription([
        tf_ned_enu,
        tf_odom_ned,
        tf_base_cam_left,
        tf_base_cam_right,
        tf_multibeam,
        rviz_node,
        robot_description,
        launch_include,
        # ardusim_bluerov2, 
        # ardusim_blueboat, 
    ])