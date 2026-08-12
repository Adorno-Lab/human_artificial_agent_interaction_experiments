from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip_address = "192.168.0.100"

    return LaunchDescription([
        # Detection of markers poses node:
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('markers'),
                    'launch',
                    'markers_node_launch.py'
                ])
            ]),
            launch_arguments={
                'show_camera': 'True',
                'draw_markers': 'True',
                'marker_size': '0.062'
            }.items()
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
                'nao_ip': robot_ip_address
            }.items()
        ),

        # Robot communication node:
        Node(
            package='robot_communication',
            executable='robot_communication_node',
            name='robot_communication_node',
            output='screen',
            parameters=[{
                # IP address of the robot.
                "robot_ip": robot_ip_address,
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
                "object11": [11, 11, 0.0, 0.0, 0.0],
                "object12": [17, 1, 0.528, -0.178, 0.0],
                "object13": [18, 1, 0.528, -0.559, 0.0],
                "object14": [19, 1, 0.528, -0.937, 0.0]
            }]
        )

    ])