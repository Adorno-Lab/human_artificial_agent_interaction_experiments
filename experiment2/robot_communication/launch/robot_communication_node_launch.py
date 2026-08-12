from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Robot IP address:
    robot_ip = LaunchConfiguration('robot_ip', default='192.168.0.100')

    return LaunchDescription([
        # Robot communication node:
        Node(
            package='robot_communication',
            executable='robot_communication_node',
            name='robot_communication_node',
            output='screen',
            parameters=[{
                # IP address of the robot.
                "robot_ip": robot_ip,
                # IP address and port to connect with the robot server.
                # Default values are '127.0.0.1' and 1111, respectively.
                "ip_robot_server": '127.0.0.1',
                "port_robot_server": 1111,  # port to connect with robot server
                # Defining objects positions relative to a marker:
                # "objectX": [objectID, marker_ID, x, y, z]
                # **Start X from 1 and increment 1 at each new object
                "object1": [1, 1, 0.0, 0.0, 0.0],
                "object2": [2, 2, 0.0, 0.0, 0.0],
                "object3": [3, 3, 0.0, 0.0, 0.0],
                "object4": [4, 4, 0.0, 0.0, 0.0],
                "object5": [5, 5, 0.0, 0.0, 0.0],
                "object6": [6, 6, 0.0, 0.0, 0.0],
                "object7": [7, 7, 0.0, 0.0, 0.0],
                "object8": [8, 8, 0.0, 0.0, 0.0],
                "object9": [9, 9, 0.0, 0.0, 0.0],
                "object10": [10, 10, 0.0, 0.0, 0.0],
                "object11": [11, 11, 0.0, 0.0, 0.0]
            }]
        ),

        # NAOqi driver for ROS 2:
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('naoqi_driver'),
                    'launch',
                    'naoqi_driver.launch.py'
                ])
            ]),
            launch_arguments={
                'nao_ip': robot_ip
            }.items()
        )
    ])