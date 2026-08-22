"""Turn an RViz goal into a /plan the MPC can follow.

nav2's bt_navigator normally does this, but it drives controller_server too,
which fights the MPC for /cmd_vel. This node talks to planner_server directly:
it takes /goal_pose, asks for a global path, and republishes it on /plan so the
path keeps up with the map as SLAM explores.
"""

import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose

REPLAN_PERIOD = 1.0    # seconds between replans while driving
GOAL_TOL = 0.10        # metres, matches the MPC's own tolerance


class GoalToPlan(Node):

    def __init__(self):
        super().__init__('goal_to_plan')

        self.goal = None
        self.position = None
        self.busy = False

        self.client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.pub = self.create_publisher(Path, '/plan', 10)
        self.create_timer(REPLAN_PERIOD, self.replan)

        self.get_logger().info('goal_to_plan ready -- send a goal from RViz')

    def on_goal(self, msg):
        self.goal = msg
        self.get_logger().info(
            f'goal ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')
        self.replan()

    def on_odom(self, msg):
        self.position = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def reached_goal(self):
        if self.goal is None or self.position is None:
            return False
        return math.hypot(self.goal.pose.position.x - self.position[0],
                          self.goal.pose.position.y - self.position[1]) < GOAL_TOL

    def replan(self):
        if self.goal is None or self.busy:
            return

        if self.reached_goal():
            # Empty path tells the MPC to stop asking for commands
            self.pub.publish(Path())
            self.goal = None
            self.get_logger().info('goal reached, planning stopped')
            return

        if not self.client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('planner_server not available')
            return

        request = ComputePathToPose.Goal()
        request.goal = self.goal
        request.use_start = False          # plan from the robot's current pose
        self.busy = True
        self.client.send_goal_async(request).add_done_callback(self.on_accepted)

    def on_accepted(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('planner rejected the goal')
            self.busy = False
            return
        handle.get_result_async().add_done_callback(self.on_result)

    def on_result(self, future):
        path = future.result().result.path
        self.busy = False
        if not path.poses:
            self.get_logger().warn('planner returned an empty path')
            return
        self.pub.publish(path)


def main():
    rclpy.init()
    node = GoalToPlan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
