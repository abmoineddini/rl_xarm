#!/usr/bin/env python3
#
# Ignition Fortress (gz-sim 6) top-level launch: xarm6 + tennis-ball gantry in
# ONE gz world. This is the "new-gazebo" counterpart of the Classic
# launch_xarm6_tennis_ball_in_gazebo.launch.py.
#
# Scope (chosen deliberately): "arm + controllers only" -- the xarm6 is spawned
# with ign_ros2_control and its joint controllers (joint_state_broadcaster +
# xarm6_traj_controller). No MoveIt / move_group / RViz are started (unlike the
# Classic launch, which pulled in the full xarm_moveit stack).
#
# What comes up:
#   1. The Fortress world + tennis-ball gantry + its ros_gz bridge, by INCLUDING
#      the existing, working launch_tennis_ball_in_gz.launch.py (world name
#      "tennis_world", ground plane, ball spawn, /clock + /joint_states +
#      /set_joint_trajectory bridge).
#   2. xarm6 spawned into that SAME world via ros_gz_sim `create`, with
#      ign_ros2_control/IgnitionSystem, at the Classic base pose (on-table
#      height z=1.021, fixed base -> no physical table needed).
#   3. joint_state_broadcaster + xarm6_traj_controller loaded once the arm is
#      spawned (drive the arm by publishing to /xarm6_traj_controller/...).
#   4. Tennis-ball helper nodes that the Classic tennis launch also ran:
#      image tracker, xarm_joint_controller, reset_sim.
#
# Why self-contained (not reusing xarm's _robot_beside_table_gazebo): on this
# machine Fortress is driven through ros_gz_sim (gz_version=6), and ros_ign_gazebo
# is NOT installed, so that launch's 'ign' branch can't run and its 'gz' branch
# uses Garden-style names. Matching the proven tennis-ball Fortress stack is the
# robust path.
#
# NOTE on the two arm-command helpers (xarm_joint_controller.py / reset):
#   They command the arm through the xarm planner services /xarm_joint_plan and
#   /xarm_exec_plan, which are part of the MoveIt stack that is intentionally
#   NOT launched here. They sit idle until triggered; when triggered without a
#   planner running they will just fail the service call. Set
#   `enable_arm_command_helpers:=false` to leave them out, or run a planner.

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
    OpaqueFunction,
    DeclareLaunchArgument,
    RegisterEventHandler,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from uf_ros_lib.uf_robot_utils import get_xacro_content, generate_ros2_control_params_temp_file


def launch_setup(context, *args, **kwargs):
    pkg_name = 'xarm6_control_trainig'
    pkg_share = get_package_share_directory(pkg_name)

    enable_arm_command_helpers = LaunchConfiguration('enable_arm_command_helpers')

    # ---- xarm6 identity (fixed for this task) ----
    dof = '6'
    robot_type = 'xarm'
    prefix = ''
    hw_ns = 'xarm'
    xarm_type = 'xarm6'
    ros2_control_plugin = 'ign_ros2_control/IgnitionSystem'  # Fortress control

    # ------------------------------------------------------------------
    # 1. World + tennis-ball gantry + ball bridge (proven Fortress launch)
    # ------------------------------------------------------------------
    tennis_ball_gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'launch_tennis_ball_in_gz.launch.py')),
    )

    # ------------------------------------------------------------------
    # 2. xarm6 description (Fortress ros2_control plugin + realsense d435i)
    # ------------------------------------------------------------------
    # controller_manager params get embedded into the ign_ros2_control plugin
    # via <parameters>; this temp yaml carries the xarm6 controllers + sim time.
    ros2_control_params = generate_ros2_control_params_temp_file(
        os.path.join(get_package_share_directory('xarm_controller'),
                     'config', '{}_controllers.yaml'.format(xarm_type)),
        prefix=prefix,
        add_gripper=False,
        add_bio_gripper=False,
        ros_namespace='',
        update_rate=1000,
        use_sim_time=True,
        robot_type=robot_type,
    )

    xarm_robot_description = get_xacro_content(
        context,
        xacro_file=Path(get_package_share_directory('xarm_description')) / 'urdf' / 'xarm_device.urdf.xacro',
        dof=dof,
        robot_type=robot_type,
        prefix=prefix,
        hw_ns=hw_ns,
        add_realsense_d435i=True,
        add_d435i_links=True,
        ros2_control_plugin=ros2_control_plugin,
        ros2_control_params=ros2_control_params,
    )

    # xarm6 robot_state_publisher. This deliberately uses the DEFAULT node name
    # 'robot_state_publisher' and publishes on '/robot_description', because the
    # ign_ros2_control plugin reads the arm description from exactly that
    # node/topic by default. The tennis include was moved off these defaults
    # (tennis_ball_state_publisher / /tennis_ball_description) so there is no
    # collision.
    xarm_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True, 'robot_description': xarm_robot_description}],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
    )

    # ------------------------------------------------------------------
    # 3. Mounting stack (cupboard + cylindrical riser) + xarm6 on top.
    #    Real setup: 0.53 m cupboard, then a 0.26 m riser, then the arm base.
    # ------------------------------------------------------------------
    arm_x, arm_y = '-0.2', '-0.5'
    cupboard_height = 0.53   # m; box z-size in worlds/pedestal.sdf
    riser_height = 0.26      # m; cylinder length in worlds/pedestal.sdf
    arm_mount_z = cupboard_height + riser_height  # 0.79 m: top of the riser

    pedestal_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-file', os.path.join(pkg_share, 'worlds', 'pedestal.sdf'),
            '-name', 'robot_pedestal',
            '-x', arm_x,
            '-y', arm_y,
            '-z', str(cupboard_height / 2.0),  # cupboard is centred -> base rests on ground
        ],
        parameters=[{'use_sim_time': True}],
    )

    # xarm6 base sits on TOP of the riser (z = cupboard + riser), facing the ball.
    xarm_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'UF_ROBOT',
            '-x', arm_x,
            '-y', arm_y,
            '-z', str(arm_mount_z),
            '-Y', '1.571',
        ],
        parameters=[{'use_sim_time': True}],
    )

    # Controllers, loaded only after the arm exists in the sim.
    controller_names = ['joint_state_broadcaster', '{}_traj_controller'.format(xarm_type)]
    controller_spawners = [
        Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            arguments=[name, '--controller-manager', '/controller_manager'],
            parameters=[{'use_sim_time': True}],
        )
        for name in controller_names
    ]

    load_controllers_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=xarm_spawn,
            on_exit=controller_spawners,
        )
    )

    # ------------------------------------------------------------------
    # 4. Camera bridge for the d435i on the arm (Fortress -> ROS), remapped
    #    to the topics the tennis-ball tracker subscribes to.
    #    NOTE: Fortress camera sensor topics are the part most likely to need
    #    live tuning; the color image is the one the tracker mainly uses.
    # ------------------------------------------------------------------
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='xarm_camera_bridge',
        output='screen',
        arguments=[
            '/camera/color/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera/color/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/camera/depth@sensor_msgs/msg/Image[ignition.msgs.Image',
        ],
        remappings=[
            ('/camera/color/image_raw', '/color/image_raw'),
            ('/camera/depth', '/aligned_depth_to_color/image_raw'),
        ],
        parameters=[{'use_sim_time': True}],
    )

    # ------------------------------------------------------------------
    # 5. Tennis-ball helper nodes (as in the Classic tennis launch). The
    #    tracker is simulator-agnostic; the two arm-command helpers need the
    #    MoveIt planner services (see header note).
    # ------------------------------------------------------------------
    image_tracker = Node(
        package='image_processing',
        executable='tennis_ball_tracker_sim_publisher.py',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    xarm_joint_controller = Node(
        package=pkg_name,
        executable='xarm_joint_controller.py',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(enable_arm_command_helpers),
    )
    reset_sim = Node(
        package=pkg_name,
        executable='reset_sim_tennis_ball.py',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(enable_arm_command_helpers),
    )

    # Give the tennis include time to bring the gz server up before spawning
    # the arm into it (mirrors the 5 s delay the Classic top-level used).
    delayed_arm = TimerAction(
        period=5.0,
        actions=[pedestal_spawn, xarm_spawn, camera_bridge],
    )
    delayed_helpers = TimerAction(
        period=8.0,
        actions=[image_tracker, xarm_joint_controller, reset_sim],
    )

    return [
        tennis_ball_gz,
        xarm_rsp,
        delayed_arm,
        load_controllers_after_spawn,
        delayed_helpers,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_arm_command_helpers',
            default_value='true',
            description='Run xarm_joint_controller.py / reset_sim_tennis_ball.py. '
                        'These need the xarm planner services (/xarm_joint_plan, '
                        '/xarm_exec_plan) from the MoveIt stack, which is NOT '
                        'launched here. Set false to omit them.',
        ),
        OpaqueFunction(function=launch_setup),
    ])
