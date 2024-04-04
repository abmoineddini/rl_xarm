import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import time


from launch_ros.actions import Node
import xacro


def generate_launch_description():

    add_realsense_d435i = LaunchConfiguration('add_realsense_d435i', default=True)
    add_link_realsense_d435i = LaunchConfiguration('add_realsense_d435i', default=True)

    Xarm6_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_planner'), 'launch', 'xarm6_planner_gazebo.launch.py'])),
        launch_arguments={
            'add_realsense_d435i': add_realsense_d435i,
            'add_d435i_links': add_link_realsense_d435i,
        }.items(),
    )

    target_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm6_control_trainig'), 'launch', 'launch_tennis_ball_in_gazebo.launch.py'])),
    )

    delayed_launch = TimerAction(period=5.0, actions=[target_launcher])



    # Run the node
    return LaunchDescription([
        Xarm6_launch,
        delayed_launch
    ])