#!/usr/bin/env python3
#
# Randomly moves the tennis ball around by driving the CYLINDRICAL gantry
# joints (theta_joint / r_joint / z_joint) via /set_joint_trajectory, which the
# gz JointTrajectoryController (or the Classic joint_pose_trajectory plugin)
# consumes.
#
# Motion model: each joint INDEPENDENTLY seeks a random target drawn uniformly
# from its full range. When it reaches the target it picks a new one (at a new
# random speed). Because targets are uniform over the whole range and the three
# joints are unsynchronised, the ball roams the entire reachable volume instead
# of piling up at the limits.
#
# History: the very first version did `pos = sin(pos + v*dt)` (decays to 0, ball
# stops). The second used a reflecting velocity random walk, which tended to
# accumulate at the joint limits (ball stuck far left/right). This target-
# seeking version spreads the motion out much more evenly.
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
        # start each joint at a random spot in its range
        self.pos = {j: random.uniform(lo, hi) for j, (lo, hi) in JOINT_LIMITS.items()}
        self.target = {}
        self.speed = {}
        for j in JOINTS:
            self._new_target(j)

        self.create_timer(self.dt, self.timer_callback)

    def _new_target(self, joint):
        lo, hi = JOINT_LIMITS[joint]
        span = hi - lo
        # Uniform target over the FULL range -> even coverage of the space.
        self.target[joint] = random.uniform(lo, hi)
        # Random traverse speed each leg, so the pace varies too.
        self.speed[joint] = random.uniform(0.3, 1.0) * span  # units/sec

    def timer_callback(self):
        for joint in JOINTS:
            step = self.speed[joint] * self.dt
            error = self.target[joint] - self.pos[joint]
            if abs(error) <= step:
                # reached (or overshoot this tick) -> snap and pick a new target
                self.pos[joint] = self.target[joint]
                self._new_target(joint)
            else:
                self.pos[joint] += step if error > 0 else -step

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
