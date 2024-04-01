#!/usr/bin/env python3
import sys
from xarm_msgs.srv import PlanExec
from xarm_msgs.srv import PlanJoint
from std_msgs.msg import Empty
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import rclpy
from rclpy.node import Node


class ResetSim(Node):
    def __init__(self):
        super().__init__('reset_sim')
        # Reseting the arm
        self.subscription_colour_image = self.create_subscription(Empty, 'reset', self.callback, 10)
        
        self.execute_joint_pos = self.create_client(PlanExec, '/xarm_exec_plan')
        self.plan_joint_pos = self.create_client(PlanJoint, '/xarm_joint_plan')

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
        future = self.execute_joint_pos.call_async(self.req_excute)
        # rclpy.spin_until_future_complete(self, future)
        return future.result()
    
    def send_plan_request(self):
        target_initial_pose = [-8.0, -55, -18.0, 5.0, 46.0, -11.0]
        print([x * 0.01745329251 for x in target_initial_pose])
        self.req_plan.target = [x * 0.01745329251 for x in target_initial_pose]
        future = self.plan_joint_pos.call_async(self.req_plan)
        return future.result()

    def reset_ball_pose(self):
        self.publisher_.publish(self.msg)

    
    def callback(self, msg):
        # self.reset_ball_pose()

        response = self.send_plan_request()
        while not hasattr(response, 'success'):
            print("waiting for completion")
        self.send_execute_request()
        

        # if result.sucess:


def main(args=None):
    rclpy.init(args=args)
    node = ResetSim()

    rclpy.spin(node)
    node.shutdown()
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()