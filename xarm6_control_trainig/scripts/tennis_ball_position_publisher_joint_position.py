#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import random

class PositionPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(JointTrajectory, 'set_joint_trajectory', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.x = 0
        self.y = 0
        self.z = 0

    def timer_callback(self):
        msg = JointTrajectory()
        dx = random.randint(-10,10) 
        dy = random.randint(-10,10) 
        dz = random.randint(-10,10) 
        self.x = self.x+dx/300  
        self.y = self.y+dy/300
        self.z = self.z+dz/300
        msg.header.frame_id = 'world'
        msg.joint_names = ['x_joint', 'y_joint', 'z_joint']
        trajectory_point = JointTrajectoryPoint()
        trajectory_point.positions = [self.x, self.y, self.z]  # Joint positions
        trajectory_point.velocities = [0.0, 0.0, 0.0]  # Joint velocities
        trajectory_point.accelerations = [0.0, 0.0, 0.0]  # Joint accelerations
        msg.points.append(trajectory_point)
        self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)
        # self.i += 1


def main(args=None):
    rclpy.init(args=args)

    tennis_ball_position_publisher = PositionPublisher()

    rclpy.spin(tennis_ball_position_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    tennis_ball_position_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
#ros2 topic pub -1 /set_joint_trajectory trajectory_msgs/msg/JointTrajectory  '{header: {frame_id: world}, joint_names: [x_joint, y_joint, z_joint], points: [{positions: {0.3,0.3,0.3}, velocities: {0.02,0.01,0.01}}}]}
