"""Drive namespaced robots to goals one at a time.

Serial by design: robot r1 is sent a goal and driven to completion, then r2, then
r3, then round again. Only one robot moves at any moment, so there is no
inter-robot deconfliction to do -- no traffic manager, no priorities, no yielding.

Whether a 2x2 m room can hold 2-3 of these robots shows up in the printed summary:
a goal that fails or times out is the room telling you it is too tight.

    ros2 run nav2_mobile_robot serial_driver --ros-args \
        -p robots:="[r1, r2]" -p rounds:=3
"""

import math
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

# Goal poses must NOT be the spawn/parking poses in multi_gazebo.launch.py: a
# parked robot sits on its spawn pose, and sending a peer there just rams it.
# These are the four mid-edge points instead -- every one has >=0.225 m clearance
# (nav footprint is 0.18 m) and is >=0.70 m from any parking spot, while still
# forcing each robot to cross the room past both boxes.
DEFAULT_WAYPOINTS = [
    0.00, 0.75, -1.571,
    0.75, 0.00, 3.142,
    0.00, -0.75, 1.571,
    -0.75, 0.00, 0.000,
]

MAP_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
)


class SerialDriver(Node):
    def __init__(self):
        super().__init__("serial_driver")

        self.declare_parameter("robots", ["r1", "r2"])
        self.declare_parameter("rounds", 2)
        self.declare_parameter("waypoints", DEFAULT_WAYPOINTS)
        self.declare_parameter("goal_timeout", 90.0)
        self.declare_parameter("settle_time", 1.0)
        # Two robot bodies are 0.30 m across; leave margin on top of that.
        self.declare_parameter("min_peer_distance", 0.45)

        self.robots = list(self.get_parameter("robots").value)
        self.rounds = int(self.get_parameter("rounds").value)
        self.goal_timeout = float(self.get_parameter("goal_timeout").value)
        self.settle_time = float(self.get_parameter("settle_time").value)
        self.min_peer_distance = float(self.get_parameter("min_peer_distance").value)

        flat = list(self.get_parameter("waypoints").value)
        self.waypoints = [tuple(flat[i : i + 3]) for i in range(0, len(flat), 3)]

        self.grid = None
        self.create_subscription(OccupancyGrid, "/map", self._on_map, MAP_QOS)

        # Robots park on whatever goal they last reached, so a waypoint can be
        # occupied by the time it comes round again. Track poses and skip those.
        self.peer_poses = {}
        for peer in self.robots:
            self.create_subscription(
                Odometry, f"/{peer}/odom", self._make_odom_cb(peer), 10
            )

        self.nav_clients = {
            name: ActionClient(self, NavigateToPose, f"/{name}/navigate_to_pose")
            for name in self.robots
        }
        # Peer outlines are marked into the costmaps but never raytrace-cleared,
        # so wipe both before each goal rather than let stale marks accumulate.
        self.clearers = {
            name: (
                self.create_client(
                    ClearEntireCostmap,
                    f"/{name}/global_costmap/clear_entirely_global_costmap",
                ),
                self.create_client(
                    ClearEntireCostmap,
                    f"/{name}/local_costmap/clear_entirely_local_costmap",
                ),
            )
            for name in self.robots
        }

        # name -> [attempted, succeeded, total_seconds]
        self.stats = {name: [0, 0, 0.0] for name in self.robots}

    def _make_odom_cb(self, peer):
        def cb(msg):
            self.peer_poses[peer] = (
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
            )
        return cb

    def _pick_goal(self, index, rnd, name):
        """First waypoint in rotation that no other robot is parked on."""
        count = len(self.waypoints)
        for offset in range(count):
            x, y, yaw = self.waypoints[(index + rnd + 1 + offset) % count]
            blocked = next(
                (
                    peer
                    for peer, (px, py) in self.peer_poses.items()
                    if peer != name
                    and math.hypot(px - x, py - y) < self.min_peer_distance
                ),
                None,
            )
            if blocked is None:
                return x, y, yaw
            self.get_logger().info(
                f"{name}: skipping ({x:+.2f}, {y:+.2f}) -- {blocked} is parked there"
            )
        return None

    def _on_map(self, msg):
        if self.grid is None:
            self.grid = msg
            self.get_logger().info(
                f"map: {msg.info.width}x{msg.info.height} @ {msg.info.resolution:.4f} m/px"
            )

    # ---------- goal helpers ----------

    def _pose(self, x, y, yaw):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        return goal

    def _await(self, future, timeout):
        deadline = time.time() + timeout
        while rclpy.ok() and not future.done() and time.time() < deadline:
            time.sleep(0.05)
        return future.result() if future.done() else None

    def _clear_costmaps(self, name):
        for client in self.clearers[name]:
            if client.wait_for_service(timeout_sec=3.0):
                self._await(client.call_async(ClearEntireCostmap.Request()), 3.0)
            else:
                self.get_logger().warn(f"{name}: {client.srv_name} unavailable")

    def _drive(self, name, x, y, yaw):
        """Send one goal and block until the robot finishes it."""
        self._clear_costmaps(name)
        client = self.nav_clients[name]
        if not client.wait_for_server(timeout_sec=20.0):
            self.get_logger().error(f"{name}: no navigate_to_pose server")
            return False

        goal = NavigateToPose.Goal()
        goal.pose = self._pose(x, y, yaw)

        self.stats[name][0] += 1
        started = time.time()
        self.get_logger().info(f"{name} -> ({x:+.2f}, {y:+.2f})")

        handle = self._await(client.send_goal_async(goal), 15.0)
        if handle is None or not handle.accepted:
            self.get_logger().error(f"{name}: goal rejected")
            return False

        result = self._await(handle.get_result_async(), self.goal_timeout)
        elapsed = time.time() - started
        self.stats[name][2] += elapsed

        if result is None:
            self.get_logger().error(f"{name}: TIMEOUT after {elapsed:.1f}s -- cancelling")
            self._await(handle.cancel_goal_async(), 5.0)
            return False

        # status 4 == STATUS_SUCCEEDED
        if result.status == 4:
            self.stats[name][1] += 1
            self.get_logger().info(f"{name}: reached goal in {elapsed:.1f}s")
            return True

        self.get_logger().warn(f"{name}: FAILED (status {result.status}) after {elapsed:.1f}s")
        return False

    # ---------- main sequence ----------

    def run(self):
        self.get_logger().info(f"waiting for /map ...")
        while rclpy.ok() and self.grid is None:
            time.sleep(0.2)

        n = len(self.robots)
        for rnd in range(self.rounds):
            self.get_logger().info(f"===== round {rnd + 1}/{self.rounds} =====")
            for i, name in enumerate(self.robots):
                # Rotate targets so every robot crosses the room, not just nudges.
                goal = self._pick_goal(i, rnd, name)
                if goal is None:
                    self.get_logger().warn(f"{name}: every waypoint occupied, skipping")
                    continue
                self._drive(name, *goal)
                time.sleep(self.settle_time)
        self.report()

    def report(self):
        self.get_logger().info("===== summary =====")
        self.get_logger().info(f"{'robot':8s} {'goals':>6s} {'ok':>4s} {'failed':>7s} {'mean s':>8s}")
        for name, (attempted, ok, total) in self.stats.items():
            mean = total / attempted if attempted else 0.0
            self.get_logger().info(
                f"{name:8s} {attempted:6d} {ok:4d} {attempted - ok:7d} {mean:8.1f}"
            )


def main():
    rclpy.init()
    node = SerialDriver()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    try:
        node.run()
    except KeyboardInterrupt:
        node.report()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
