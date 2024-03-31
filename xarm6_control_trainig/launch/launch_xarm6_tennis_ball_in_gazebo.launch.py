import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
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


    time.sleep(3)
    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'xarm6_control_trainig'
    file_subpath = 'urdf/tennis_robot.urdf.xacro'


    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()


    # Configure the node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
        'use_sim_time': True}] # add other parameters here if required
    )


    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                                '-entity', 'tennis_ball'],
                    output='screen')




    # Run the node
    return LaunchDescription([
        Xarm6_launch,
        node_robot_state_publisher,
        spawn_entity
    ])