"""Follow /plan with an acados MPC and publish /cmd_vel.

Standalone controller: run it INSTEAD of nav2's controller_server, which
publishes to the same /cmd_vel chain.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist

from mpc_control.mpc_model import build_solver, unwrap, N

CONTROL_PERIOD = 0.05     # 20 Hz, same rate as nav2's controller_frequency
CRUISE_SPEED = 0.15       # reference forward speed in the cost
GOAL_TOL = 0.10           # metres, matches xy_goal_tolerance in nav.yaml


class MpcControl(Node):

    def __init__(self):
        super().__init__('mpc_control')

        self.solver = build_solver()
        self.state = np.zeros(3)
        self.path = []

        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(Path, '/plan', self.on_path, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(CONTROL_PERIOD, self.control_step)

        self.get_logger().info('acados MPC ready')

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        self.state = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                       1.0 - 2.0 * (q.y * q.y + q.z * q.z)),
        ])

    def on_path(self, msg):
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def closest_index(self):
        d = [math.hypot(px - self.state[0], py - self.state[1])
             for px, py in self.path]
        return int(np.argmin(d))

    def reference_at(self, start, k):
        """Waypoint k steps past the closest point on the path."""
        i = min(start + k, len(self.path) - 1)
        return np.array(self.path[i])

    def stop(self):
        self.pub.publish(Twist())

    def control_step(self):
        if not self.path:
            return

        goal = np.array(self.path[-1])
        if math.hypot(goal[0] - self.state[0], goal[1] - self.state[1]) < GOAL_TOL:
            self.stop()
            self.path = []
            self.get_logger().info('goal reached')
            return

        # Current state is the initial condition of the horizon
        self.solver.set(0, 'lbx', self.state)
        self.solver.set(0, 'ubx', self.state)

        start = self.closest_index()
        near = self.state[2]
        for k in range(N):
            ref = self.reference_at(start, k)
            nxt = self.reference_at(start, k + 1)
            yaw = math.atan2(nxt[1] - ref[1], nxt[0] - ref[0])
            # Keep the reference yaw within pi of the previous one, so the
            # solver never sees a 2*pi jump and turns the short way round.
            near = unwrap(yaw, near)
            # yref = [x, y, theta, v, w]
            self.solver.set(k, 'yref',
                            np.array([ref[0], ref[1], near, CRUISE_SPEED, 0.0]))

        ref_e = self.reference_at(start, N)
        self.solver.set(N, 'yref', np.array([ref_e[0], ref_e[1], near]))

        status = self.solver.solve()
        if status != 0:
            self.get_logger().warn(f'solver status {status}, stopping')
            self.stop()
            return

        u = self.solver.get(0, 'u')
        cmd = Twist()
        cmd.linear.x = float(u[0])
        cmd.angular.z = float(u[1])
        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = MpcControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.stop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
