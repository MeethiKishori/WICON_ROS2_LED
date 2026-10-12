import math
import signal
import threading

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.signals import SignalHandlerOptions


COSTMAP_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
)

IDLE = "IDLE"
DRIVING = "DRIVING"
DWELL = "DWELL"
ESCAPING = "ESCAPING"


class Robot:
    def __init__(self, node, name, route):
        self.node = node
        self.name = name
        self.route = route

        self.state = IDLE
        self.state_since = 0.0
        self.index = 0
        self.pose = None
        self.target = None
        self.goal_handle = None
        self.waiting_since = None

        self.legs = 0
        self.laps = 0
        self.failed = 0

        self.costmap = None
        self.trapped_since = None
        self.escapes = 0

        self.nav_client = ActionClient(node, NavigateToPose, f"/{name}/navigate_to_pose")
        node.create_subscription(Odometry, f"/{name}/odom", self.on_odom, 10)
        node.create_subscription(
            OccupancyGrid, f"/{name}/global_costmap/costmap", self.on_costmap, COSTMAP_QOS
        )

    def on_odom(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.pose = (x, y)

    def on_costmap(self, msg):
        self.costmap = msg

    def set_state(self, state):
        self.state = state
        self.state_since = self.node.now_seconds()

    def next_waypoint(self):
        position = self.index % len(self.route)
        return self.route[position]

    def cancel_goal(self):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None

    def cost(self, x, y):
        grid = self.costmap
        if grid is None:
            return None

        cell_x = int((x - grid.info.origin.position.x) / grid.info.resolution)
        cell_y = int((y - grid.info.origin.position.y) / grid.info.resolution)

        inside_x = 0 <= cell_x < grid.info.width
        inside_y = 0 <= cell_y < grid.info.height
        if inside_x and inside_y:
            return grid.data[cell_y * grid.info.width + cell_x]
        return None

    def cost_here(self):
        if self.pose is None:
            return None
        x, y = self.pose
        return self.cost(x, y)

    def is_free_around(self, x, y):
        for angle in (0, math.pi / 2, math.pi, 3 * math.pi / 2):
            nearby_x = x + 0.12 * math.cos(angle)
            nearby_y = y + 0.12 * math.sin(angle)
            if self.cost(nearby_x, nearby_y) != 0:
                return False
        return True

    def way_out(self, search, step=0.05):
        if self.pose is None or self.costmap is None:
            return None

        start_x, start_y = self.pose
        ring_count = int(search / step)

        for ring in range(2, ring_count + 1):
            radius = ring * step
            points_on_ring = int(2 * math.pi * radius / step)

            for k in range(points_on_ring):
                angle = 2 * math.pi * k / max(1, points_on_ring)
                x = start_x + radius * math.cos(angle)
                y = start_y + radius * math.sin(angle)

                if self.cost(x, y) != 0:
                    continue
                if self.is_free_around(x, y):
                    return x, y

        return None


class FleetManager(Node):
    def __init__(self):
        super().__init__("fleet_manager", automatically_declare_parameters_from_overrides=True)

        self.lock = threading.RLock()

        self.dwell = self.get_param("dwell", 1.5)
        self.loop = self.get_param("loop", True)
        self.min_separation = self.get_param("min_separation", 0.55)
        self.goal_timeout = self.get_param("goal_timeout", 120.0)
        self.retry_delay = self.get_param("retry_delay", 3.0)
        self.tick_period = self.get_param("tick_period", 1.0)
        self.escape_cost = int(self.get_param("escape_cost", 50))
        self.escape_after = self.get_param("escape_after", 5.0)
        self.escape_search = self.get_param("escape_search", 0.70)
        self.escape_attempts = int(self.get_param("escape_attempts", 3))

        self.waypoints = {}
        for name, param in self.get_parameters_by_prefix("platforms").items():
            self.waypoints[name] = tuple(float(v) for v in param.value)

        routes = {}
        for name, param in self.get_parameters_by_prefix("routes").items():
            routes[name] = list(param.value)

        if not self.waypoints or not routes:
            raise RuntimeError("no platforms/routes -- launch via fleet.launch.py")

        selected = []
        for name in self.get_param("robots", []):
            if name:
                selected.append(name)

        robot_names = []
        for name in sorted(routes):
            if not selected or name in selected:
                robot_names.append(name)

        unknown = set()
        for name in robot_names:
            for waypoint in routes[name]:
                if waypoint not in self.waypoints:
                    unknown.add(waypoint)
        if unknown:
            raise RuntimeError(f"routes use undefined waypoints: {sorted(unknown)}")

        self.robots = []
        for name in robot_names:
            self.robots.append(Robot(self, name, routes[name]))

        self.started = False
        self.create_timer(self.tick_period, self.on_tick)

        for robot in self.robots:
            self.get_logger().info(f"{robot.name}: {' -> '.join(robot.route)}")
        self.get_logger().info(
            f"keeping destinations {self.min_separation:.2f} m apart; Nav2 does the rest"
        )

    def get_param(self, name, default):
        if not self.has_parameter(name):
            self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def all_robots_ready(self):
        for robot in self.robots:
            if robot.pose is None:
                return False

        for robot in self.robots:
            if not robot.nav_client.server_is_ready():
                self.get_logger().info("waiting for Nav2 ...", throttle_duration_sec=10.0)
                return False

        return True

    def who_has_spot(self, robot, spot):
        for other in self.robots:
            if other is robot:
                continue

            if other.pose and math.dist(other.pose, spot) < self.min_separation:
                return f"{other.name} is there"

            if other.target:
                other_goal = self.waypoints[other.target][:2]
                if math.dist(other_goal, spot) < self.min_separation:
                    return f"{other.name} is heading there"

        return None

    def on_tick(self):
        with self.lock:
            if not self.started:
                if not self.all_robots_ready():
                    return
                self.started = True
                self.get_logger().info("all robots up -- running waypoints")
                for robot in self.robots:
                    robot.state_since = self.now_seconds()

            now = self.now_seconds()

            def waiting_time(robot):
                if robot.waiting_since is None:
                    return now
                return robot.waiting_since

            robots_in_order = sorted(self.robots, key=waiting_time)

            for robot in robots_in_order:
                self.update_robot(robot, now)

    def update_robot(self, robot, now):
        if self.handle_trapped(robot, now):
            return

        if robot.state == DWELL and now - robot.state_since >= self.dwell:
            robot.set_state(IDLE)

        if robot.state == DRIVING and now - robot.state_since > self.goal_timeout:
            self.get_logger().warn(f"{robot.name}: giving up on {robot.target}")
            robot.cancel_goal()
            self.finish_leg(robot, success=False)

        if robot.state != IDLE:
            return

        spot = self.waypoints[robot.next_waypoint()][:2]
        reason = self.who_has_spot(robot, spot)
        if reason:
            if robot.waiting_since is None:
                robot.waiting_since = now
                self.get_logger().info(
                    f"{robot.name}: waiting for {robot.next_waypoint()} -- {reason}"
                )
            return

        robot.waiting_since = None
        self.send_to_next_waypoint(robot)

    def handle_trapped(self, robot, now):
        cost = robot.cost_here()
        if cost is None:
            return robot.state == ESCAPING

        if cost >= self.escape_cost:
            if robot.trapped_since is None:
                robot.trapped_since = now
        elif cost == 0:
            robot.trapped_since = None

        if robot.state == ESCAPING:
            return self.check_escape_progress(robot, now, cost)

        if robot.trapped_since is None:
            return False
        if now - robot.trapped_since < self.escape_after:
            return False

        spot = robot.way_out(self.escape_search)
        if spot is None:
            self.get_logger().warn(
                f"{robot.name}: stuck at cost {cost} and nowhere free within "
                f"{self.escape_search:.2f} m"
            )
            robot.trapped_since = now
            return False

        self.get_logger().warn(
            f"{robot.name}: inside the inflation ring (cost {cost}) for "
            f"{now - robot.trapped_since:.0f}s -- backing out to "
            f"({spot[0]:+.2f}, {spot[1]:+.2f}) before carrying on"
        )
        robot.cancel_goal()
        robot.target = None
        self.send_goal(robot, spot[0], spot[1], 0.0, ESCAPING)
        return True

    def check_escape_progress(self, robot, now, cost):
        if cost == 0:
            self.get_logger().info(f"{robot.name}: clear again, resuming")
            robot.escapes = 0
            robot.cancel_goal()
            robot.target = None
            robot.set_state(IDLE)
            return False

        if now - robot.state_since > self.escape_after * 2:
            robot.escapes += 1
            robot.cancel_goal()

            if robot.escapes >= self.escape_attempts:
                robot.escapes = 0
                robot.trapped_since = None
                robot.index += 1
                self.get_logger().error(
                    f"{robot.name}: cannot get clear -- skipping to "
                    f"{robot.next_waypoint()}. It may need moving by hand."
                )

            robot.target = None
            robot.set_state(IDLE)
            return False

        return True

    def send_to_next_waypoint(self, robot):
        robot.target = robot.next_waypoint()
        x, y, yaw = self.waypoints[robot.target]
        self.get_logger().info(f"{robot.name} -> {robot.target} ({x:+.2f}, {y:+.2f})")
        self.send_goal(robot, x, y, yaw, DRIVING)

    def send_goal(self, robot, x, y, yaw, state):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        robot.set_state(state)

        def on_goal_response(future):
            self.on_goal_response(future, robot)

        future = robot.nav_client.send_goal_async(goal)
        future.add_done_callback(on_goal_response)

    def on_goal_response(self, future, robot):
        goal_handle = future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn(f"{robot.name}: goal rejected")
            with self.lock:
                self.finish_leg(robot, success=False)
            return

        with self.lock:
            robot.goal_handle = goal_handle

        def on_result(result_future):
            self.on_goal_result(result_future, robot)

        goal_handle.get_result_async().add_done_callback(on_result)

    def on_goal_result(self, future, robot):
        result = future.result()

        with self.lock:
            robot.goal_handle = None

            if robot.state == ESCAPING:
                robot.set_state(ESCAPING)
                return

            if robot.state != DRIVING:
                return

            succeeded = result is not None and result.status == GoalStatus.STATUS_SUCCEEDED
            if succeeded:
                took = self.now_seconds() - robot.state_since
                self.get_logger().info(f"{robot.name}: reached {robot.target} in {took:.1f}s")
                self.finish_leg(robot, success=True)
            else:
                self.get_logger().info(
                    f"{robot.name}: could not reach {robot.target}, retrying"
                )
                self.finish_leg(robot, success=False)

    def finish_leg(self, robot, success):
        robot.target = None

        if success:
            robot.legs += 1
            robot.index += 1
            if robot.index % len(robot.route) == 0:
                robot.laps += 1
                self.get_logger().info(f"{robot.name}: lap {robot.laps} complete")
            robot.set_state(DWELL)
        else:
            robot.failed += 1
            robot.set_state(DWELL)
            robot.state_since = self.now_seconds() - self.dwell + self.retry_delay

    def stop(self):
        with self.lock:
            for robot in self.robots:
                robot.cancel_goal()
        self.get_logger().info("stopped")

    def report(self):
        self.get_logger().info(f"{'robot':8s} {'legs':>5s} {'laps':>5s} {'retries':>8s}")
        for robot in self.robots:
            self.get_logger().info(
                f"{robot.name:8s} {robot.legs:5d} {robot.laps:5d} {robot.failed:8d}"
            )


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = FleetManager()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    finished = threading.Event()

    def on_shutdown_signal(signum, frame):
        if finished.is_set():
            return
        finished.set()
        node.stop()
        node.report()
        executor.shutdown(timeout_sec=0.0)
        if rclpy.ok():
            rclpy.shutdown()

    signal.signal(signal.SIGINT, on_shutdown_signal)
    signal.signal(signal.SIGTERM, on_shutdown_signal)

    try:
        executor.spin()
    except Exception:
        pass
    finally:
        if not finished.is_set():
            node.stop()
            node.report()


if __name__ == "__main__":
    main()
