#!/usr/bin/env python3
import sys
from xarm_msgs.srv import PlanExec
from xarm_msgs.srv import PlanJoint
from std_msgs.msg import Empty, Float32MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

import numpy as np


class ResetSim(Node):
    def __init__(self):
        super().__init__('reset_sim_node')

        self.subscription_colour_image = self.create_subscription(Empty, 'reset_sim', self.callback, 10) #, callback_group=topic_group)
        self.publisher_joint_controller_ = self.create_publisher(Float32MultiArray, 'xarm_joint_controller', 10)
        


        target_initial_pose = [-0.13962634008, -0.9599310880499999, -0.31415926518, 0.08726646254999999, 0.8028514554599999, -0.19198621760999998]
        self.target_msg = Float32MultiArray()
        self.target_msg.data = target_initial_pose

        # Reseting the ball Pose
        self.publisher_ = self.create_publisher(JointTrajectory, 'set_joint_trajectory', 10)        
        trajectory_point = JointTrajectoryPoint()
        trajectory_point.positions = [0.0, 0.0, 0.0]  # Joint positions
        trajectory_point.velocities = [0.0, 0.0, 0.0]  # Joint velocities
        trajectory_point.accelerations = [0.0, 0.0, 0.0]  # Joint accelerations
        self.msg = JointTrajectory()
        self.msg.header.frame_id = 'world'
        self.msg.joint_names = ['theta_joint', 'r_joint', 'z_joint']
        self.msg.points.append(trajectory_point)

    
    def callback(self, Null_msg):
            print("publish reset arm message")
            self.publisher_joint_controller_.publish(self.target_msg)
            print("publish reset ball position message")
            self.publisher_.publish(self.msg)
        

        
        # if result.sucess:


def main(args=None):
    rclpy.init(args=args)
    node = ResetSim()
    # executor = MultiThreadedExecutor()
    # executor.add_node(node)
    rclpy.spin(node=node)
    node.destroy_node()
    rclpy.shutdown()

    # try:
    #     node.get_logger().info('Beginning client, shut down with CTRL-C')
    #     executor.spin()
    # except KeyboardInterrupt:
    #     node.get_logger().info('Keyboard interrupt, shutting down.\n')
    # node.destroy_node()


if __name__ == '__main__':
    main()