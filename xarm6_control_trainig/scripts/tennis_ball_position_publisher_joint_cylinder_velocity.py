#!/usr/bin/env python3
#
# Randomly moves the tennis ball around by driving the CYLINDRICAL gantry
# joints (theta_joint / r_joint / z_joint) via /set_joint_trajectory, which the
# gz JointTrajectoryController (or the Classic joint_pose_trajectory plugin)
# consumes.
#
# Each joint does a bounded random walk: it integrates a random velocity and
# bounces off its joint limit, so the ball keeps moving and never leaves the
# reachable range. Velocities are re-randomised once per second.
#
# NOTE (previous bug): the old version did `pos = sin(pos + v*dt)`. Iterating
# sin is a contraction, so all positions decayed to ~0 and the ball drifted to
# the centre and stopped. It also ignored the joint limits below.
import random

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# (lower, upper) limits — must match tennis_robot_cylindrical*.urdf.xacro
JOINT_LIMITS = {
    'theta_joint': (-1.5707, 1.5707),
    'r_joint':     (-0.35,   0.10),
    'z_joint':     (-0.20,   0.75),
}
JOINTS = ['theta_joint', 'r_joint', 'z_joint']


class PositionPublisher(Node):

    def __init__(self):
        super().__init__('tennis_ball_random_mover')
        self.publisher_ = self.create_publisher(JointTrajectory, 'set_joint_trajectory', 10)

        self.dt = 0.0332  # ~30 Hz command rate
        # start each joint at the middle of its range
        self.pos = {j: 0.5 * (lo + hi) for j, (lo, hi) in JOINT_LIMITS.items()}
        self.vel = {j: 0.0 for j in JOINTS}
        self._randomize_velocity()

        self.create_timer(self.dt, self.timer_callback)
        self.create_timer(1.0, self._randomize_velocity)  # new random speeds each second

    def _randomize_velocity(self):
        # Random signed speed per joint, scaled to each joint's span so every
        # joint moves visibly regardless of how tight its limits are.
        for j, (lo, hi) in JOINT_LIMITS.items():
            span = hi - lo
            speed = random.uniform(0.2, 1.0) * span  # units/sec (rad for theta, m for r/z)
            self.vel[j] = speed * random.choice((-1.0, 1.0))

    def timer_callback(self):
        for j, (lo, hi) in JOINT_LIMITS.items():
            p = self.pos[j] + self.vel[j] * self.dt
            # Bounce off the limits so motion stays in range and keeps going.
            if p < lo:
                p, self.vel[j] = lo, abs(self.vel[j])
            elif p > hi:
                p, self.vel[j] = hi, -abs(self.vel[j])
            self.pos[j] = p

        point = JointTrajectoryPoint()
        point.positions = [self.pos[j] for j in JOINTS]
        point.velocities = [0.0, 0.0, 0.0]
        point.accelerations = [0.0, 0.0, 0.0]

        msg = JointTrajectory()
        msg.header.frame_id = 'world'
        msg.joint_names = JOINTS
        msg.points.append(point)
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PositionPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
