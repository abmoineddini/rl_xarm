#!/usr/bin/env python3
import sys
from xarm_msgs.srv import PlanExec
from xarm_msgs.srv import PlanJoint
from std_msgs.msg import Empty
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


class ResetSim(Node):
    def __init__(self):
        super().__init__('reset_sim')

        topic_group = MutuallyExclusiveCallbackGroup()
        plan_group = MutuallyExclusiveCallbackGroup()
        excute_group = MutuallyExclusiveCallbackGroup()
        self.subscription_colour_image = self.create_subscription(Empty, 'reset', self.callback, 10, callback_group=topic_group)
        
        self.execute_joint_pos = self.create_client(PlanExec, '/xarm_exec_plan', callback_group= plan_group)
        self.plan_joint_pos = self.create_client(PlanJoint, '/xarm_joint_plan', callback_group= excute_group)

        self.req_plan = PlanJoint.Request()
        self.req_excute = PlanExec.Request()

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

    def send_execute_request(self):
        self.req_excute.wait = False     
        return self.execute_joint_pos.call(self.req_excute)
    
    def send_plan_request(self):
        target_initial_pose = [-8.0, -55, -18.0, 5.0, 46.0, -11.0]
        print([x * 0.01745329251 for x in target_initial_pose])
        self.req_plan.target = [x * 0.01745329251 for x in target_initial_pose]
        response = self.plan_joint_pos.call(self.req_plan)
        print(response)
        return response
    
    def callback(self, Null_msg):
        response = self.send_plan_request()
        print(f"group response = {response.success}")
        if response.success==True:
            response = self.send_execute_request()
            print(response)
            self.publisher_.publish(self.msg)
        

        
        # if result.sucess:


def main(args=None):
    rclpy.init(args=args)
    node = ResetSim()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        node.get_logger().info('Beginning client, shut down with CTRL-C')
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.\n')
    node.destroy_node()


if __name__ == '__main__':
    main()