#!/usr/bin/env python3
# This launch:
#   1. Starts Ignition Fortress with the minimal tennis_world.sdf world.
#   2. Starts robot_state_publisher with the new _gz description
#      (use_sim_time:=true).
#   3. Spawns the gantry+ball model into the running world via ros_gz_sim
#      `create` (reads the robot_description topic).
#   4. Starts ros_gz_bridge parameter_bridge with the yaml config so
#      /clock, /joint_states and /set_joint_trajectory are bridged.
#
# Launch arguments:
#   use_sim_time (default true) - use /clock from the Ignition sim.
#   show_frame   (default false) - render the black gantry frame. When false
#                only the tennis ball is visible; physics is identical either
#                way (collision/inertia are always present).
#
# Drive the ball (unchanged message type vs Classic):
#   ros2 topic pub -1 /set_joint_trajectory trajectory_msgs/msg/JointTrajectory \
#     '{header: {frame_id: world}, joint_names: [theta_joint, r_joint, z_joint], \
#       points: [{positions: [0.5, -0.1, 0.3]}]}'

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def launch_setup(context, *args, **kwargs):

    pkg_name = 'xarm6_control_trainig'
    pkg_share = get_package_share_directory(pkg_name)

    use_sim_time = LaunchConfiguration('use_sim_time')
    show_frame = LaunchConfiguration('show_frame').perform(context)

    # New Fortress gantry description (cylindrical variant, _gz suffix).
    # show_frame toggles the gantry frame visuals in the xacro.
    xacro_file = os.path.join(pkg_share, 'urdf',
                              'tennis_robot_cylindrical_gz.urdf.xacro')
    robot_description_raw = xacro.process_file(
        xacro_file, mappings={'show_frame': show_frame}).toxml()

    world_file = os.path.join(pkg_share, 'worlds', 'tennis_world.sdf')
    bridge_config = os.path.join(pkg_share, 'config', 'tennis_ball_gz_bridge.yaml')

    # 1. Ignition Fortress with our world. -r = run immediately (unpaused).
    #    Uses ros_gz_sim's gz_sim.launch.py. On Fortress it dispatches to
    #    `ign gazebo` because gz_version=6. We pass args through gz_args.
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': f'-r {world_file}',
            'gz_version': '6',
        }.items(),
    )

    # 2. robot_state_publisher with the new description.
    # NOTE: uses a distinct node name + description topic (not the default
    # robot_state_publisher / /robot_description) so this launch can be
    # composed with an arm whose ign_ros2_control plugin claims those defaults
    # (see launch_xarm6_tennis_ball_in_gz.launch.py). Standalone behaviour is
    # unchanged.
    robot_state_publisher = Node(
        name='tennis_ball_state_publisher',
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_raw,
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('robot_description', 'tennis_ball_description'),
            # ball joint states are bridged to /tennis_ball/joint_states (see
            # config/tennis_ball_gz_bridge.yaml) to keep the gantry joints off
            # the shared /joint_states that the arm's move_group consumes.
            ('joint_states', '/tennis_ball/joint_states'),
        ],
    )

    # 3. Spawn the model from the description topic into the world.
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'tennis_ball_description',
            '-name', 'tennis_ball',
            '-x', '0.0', '-y', '0.25', '-z', '0.0',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # 4. ros_gz_bridge parameter_bridge with the yaml config.
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='tennis_ball_gz_bridge',
        output='screen',
        parameters=[{
            'config_file': bridge_config,
            'use_sim_time': use_sim_time,
        }],
    )

    return [
        gz_sim,
        robot_state_publisher,
        spawn_entity,
        ros_gz_bridge,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use /clock from Ignition sim.'),
        DeclareLaunchArgument(
            'show_frame', default_value='false',
            description='Render the black gantry frame (true) or show only the '
                        'ball (false). Physics is unchanged either way.'),
        OpaqueFunction(function=launch_setup),
    ])
