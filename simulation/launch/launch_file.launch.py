import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf_file_name_robot = 'urdf/lynxmotion_arm.urdf'
    rviz_config_name = 'rviz/urdf.rviz'
    
    urdf_file_name_red_cup = 'urdf/redCup.urdf'

    robot_urdf = os.path.join(
        get_package_share_directory('simulation'),
        urdf_file_name_robot)
    with open(robot_urdf, 'r') as infp:
        robot_desc = infp.read()

    red_cup_urdf = os.path.join(
        get_package_share_directory('simulation'),
        urdf_file_name_red_cup)
    with open(red_cup_urdf, 'r') as infp:
        red_cup_desc = infp.read()

    rviz_config = os.path.join(
        get_package_share_directory('simulation'),
        rviz_config_name)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        Node(
            package='simulation',
            executable='cup_listener',
            name='cup_picked_up'
        ),
        Node(
            package='simulation',
            executable='robot_arm_publisher',
            name='robot_arm_publisher',
            parameters=[{'robot_description': robot_desc}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', str(rviz_config)]
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_publisher',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[robot_urdf],
        ),

        Node(
            package='simulation',
            executable='cup_publisher',
            name='cup_publisher',
            parameters=[{'use_sim_time': use_sim_time, 'red_cup': red_cup_desc}],
            arguments=[robot_urdf],
        ),
    ])