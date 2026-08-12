from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare
import math


def generate_launch_description():
    robot_ip_address = "192.168.0.107"

    l = [
        Node(
            package='interaction_manager',
            executable='manager_experiment2_node',
            name='manager_experiment2_node',
            output='screen',
            parameters=[{
                # Log information about interaction (True) or not (False).
                # Default is True
                "log": True,
                # Record video during interaction. Default is True
                "record": True,
                # Volume of the robot
                "volume": 65,
                # IP address and port to connect with the robot server.
                # Default values are '127.0.0.1' and 1111, respectively.
                "ip_robot_server": '127.0.0.1',
                "port_robot_server": 1111,
                # IP address and port to communicate with record server
                # Defaults are 192.168.0.103 and 1234, respectively
                "ip_record": "192.168.0.104",
                "port_record": 2222,
                # List of objects and their associated markers.
                # [object1_ID, object1_marker, object2_ID, object2_marker ... ]
                "objects": [1, 1,
                            2, 2,
                            3, 3,
                            4, 4,
                            5, 5,
                            6, 6,
                            7, 7,
                            8, 8,
                            9, 9,
                            10, 10,
                            11, 11,
                            12, 12,
                            13, 13,
                            14, 14,
                            15, 15
                            ],
                # Markers associated with the assembly area.
                # If the array has exactly 4 elements, each one refers
                # to a corner of the assembly area: [TL, TR, BL, BR].
                # If the array has exactly 13 elements, the first one is
                # the marker used as reference, the four next are the
                # rotation quaternion* from the reference to the corners
                # frames and the rest are the x-y coordinates of each
                # assembly area corner:
                # [marker,
                #  rw, rx, ry, rz,
                #  TL_x, TL_y, TR_x, TR_y, BL_x, BL_y, BR_x, BR_y]
                # * All corners frames are assumed to have the same
                # orientation, in a way that the assembly area is in
                # positive xy coordinates relative to the BL frame and
                # negative xy coordinates relative to TR frame.
                # (Note: write them as doubles instead of integers)
                # "assembly_area": [1.0, 2.0, 3.0, 4.0],
                "assembly_area": [16.0,
                                  math.sqrt(2)/2, 0.0, 0.0, -math.sqrt(2)/2,
                                  0.114, -0.364,
                                  0.114, -0.735,
                                  -0.257, -0.364,
                                  -0.257, -0.735],

                # Groups of objects (maximum of 10).
                # The first element is the main object that represents
                # the group. The rest are the IDs of the objects in the
                # group.
                "group1": [17, 1, 2, 3, 4, 5],
                "group2": [18, 6, 7, 8, 9, 10],
                "group3": [19, 11, 12, 13, 14, 15],

                # Color names of each group (same order as before):
                "colors": ["magenta", "green", "yellow"],

                # Sequence of objects to be indicated during phase 1:
                "sequence1": [18, 17, 18, 17, 19, 17, 18, 17, 18]
            }]
        ),

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
                'marker_size': '0.06'
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
                "port_robot_server": 1111,
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
                "object12": [12, 12, 0.0, 0.0, 0.0],
                "object13": [13, 13, 0.0, 0.0, 0.0],
                "object14": [14, 14, 0.0, 0.0, 0.0],
                "object15": [15, 15, 0.0, 0.0, 0.0],
                "object16": [16, 16, 0.0, 0.0, 0.0],
                "object17": [17, 16, 0.164, -0.168, 0.0],
                "object18": [18, 16, 0.164, -0.549, 0.0],
                "object19": [19, 16, 0.164, -0.93, 0.0]
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
                'nao_ip': robot_ip_address
            }.items()
        ),

        Node(
            package='screen_applications',
            executable='screen2_experiment2_node',
            name='screen2_experiment2_node',
            output='screen',
            parameters=[{
            }]
        )
    ]

    return LaunchDescription(l)