import os

from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare 
from launch.substitutions import PathJoinSubstitution

# # this is the function launch  system will look for
# def generate_launch_description():

#     ####### DATA INPUT ##########
#     urdf_file = 'tennis_ball.urdf'
#     #xacro_file = "urdfbot.xacro"
#     package_description = "xarm6_control_trainig"

#     ####### DATA INPUT END ##########
#     print("Fetching URDF ==>")
#     robot_desc_path = os.path.join(get_package_share_directory(package_description), "urdf", urdf_file)

#     # Robot State Publisher

#     robot_state_publisher_node = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         name='robot_state_publisher_node',
#         emulate_tty=True,
#         parameters=[{'use_sim_time': True, 'robot_description': Command(['xacro ', robot_desc_path])}],
#         output="screen"
#     )

#     # create and return launch description object
#     return LaunchDescription(
#         [            
#             robot_state_publisher_node
#         ]
#     )


import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])
    xacro_urdf_file=PathJoinSubstitution([FindPackageShare('xarm_description'), 'urdf', 'xarm_device.urdf.xacro']),
    xacro_srdf_file=PathJoinSubstitution([FindPackageShare('xarm_moveit_config'), 'srdf', 'xarm.srdf.xacro']),

    # MoveItCpp demo executable
    moveit_cpp_node = Node(
        name="controller_moveit",
        package="xarm6_control_trainig",
        executable="controller_moveit",
        output="screen",
        parameters=[{
            'robot_description':robot_description,
            'robot_description_semantic':xacro_srdf_file,
            'robot_description_kinematics':xacro_urdf_file,
            'use_sim_time': True},
        ],
    )

    return LaunchDescription(
        [moveit_cpp_node]
    )


