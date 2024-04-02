#!/usr/bin/env python3
import sys
from xarm_msgs.srv import PlanExec
from xarm_msgs.srv import PlanJoint
from std_msgs.msg import Float32MultiArray
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


class ResetSim(Node):
    def __init__(self):
        super().__init__('xarm_joint_controller_node')

        topic_group = MutuallyExclusiveCallbackGroup()
        plan_group = MutuallyExclusiveCallbackGroup()
        excute_group = MutuallyExclusiveCallbackGroup()
        self.subscription_colour_image = self.create_subscription(Float32MultiArray, 'xarm_joint_controller', self.callback, 10, callback_group=topic_group)
        
        self.execute_joint_pos = self.create_client(PlanExec, '/xarm_exec_plan', callback_group= plan_group)
        self.plan_joint_pos = self.create_client(PlanJoint, '/xarm_joint_plan', callback_group= excute_group)

        self.req_plan = PlanJoint.Request()
        self.req_excute = PlanExec.Request()


    def send_execute_request(self):
        self.req_excute.wait = True     
        return self.execute_joint_pos.call(self.req_excute)
    
    def send_plan_request(self, joint_angle):
        self.req_plan.target = joint_angle
        response = self.plan_joint_pos.call(self.req_plan)
        if response.success==True:
            response = self.send_execute_request()
            print(response)
        else:
            print("planning failed")
        return response
    
    def callback(self, msg):
        print(msg.data)
        print(type(msg.data))
        action =msg.data
        response = self.send_plan_request(action.tolist())
        print(f"group response = {response.success}")


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