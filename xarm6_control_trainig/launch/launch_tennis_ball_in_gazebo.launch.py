import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'xarm6_control_trainig'
    # file_subpath = 'urdf/tennis_robot.urdf.xacro'
    file_subpath = 'urdf/tennis_robot_cylindrical.urdf.xacro'


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

    xarm_joint_controller = Node(
        package=pkg_name,
        executable='xarm_joint_controller.py',
        output='screen',
        parameters=[{'use_sim_time': True}] # add other parameters here if required
    )

    reset_xarm_joint = Node(
        package=pkg_name,
        executable='reset_sim_tennis_ball.py',
        output='screen',
        parameters=[{'use_sim_time': True}] # add other parameters here if required
    )

    image_processing = Node(
        package='image_processing',
        executable='tennis_ball_tracker_sim_publisher.py',
        output='screen',
        parameters=[{'use_sim_time': True}] # add other parameters here if required
    )

    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
    #     )


    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                                '-entity', 'tennis_ball'],
                    output='screen')




    # Run the node
    return LaunchDescription([
        # gazebo,
        node_robot_state_publisher,
        xarm_joint_controller,
        reset_xarm_joint,
        image_processing,
        spawn_entity
    ])