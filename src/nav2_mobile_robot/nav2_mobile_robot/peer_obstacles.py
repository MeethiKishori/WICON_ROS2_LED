"""Make the robots visible to each other's costmaps.

They are invisible to lidar by construction: the scan plane sits 0.2 m above
base_link (lidar.xacro) while the tallest part of a robot reaches only 0.075 m.
So a robot drives straight through its parked neighbours -- observed directly,
r1 shoved r2 from x=0.70 to x=0.825, pinning it against the east wall.

Rather than reshape the robot so the lidar can see it, share the poses. Each
robot's odometry is world-absolute, so this node republishes every robot's
outline as a PointCloud2 in every *other* robot's costmap input. That is also
how real fleets do it -- peer awareness comes over the network, not the sensor.

    ros2 run nav2_mobile_robot peer_obstacles --ros-args -p robots:="[r1, r2]"
"""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class PeerObstacles(Node):
    def __init__(self):
        super().__init__("peer_obstacles")

        self.declare_parameter("robots", ["r1", "r2"])
        self.declare_parameter("radius", 0.16)
        self.declare_parameter("points", 24)
        self.declare_parameter("rate", 10.0)
        # Must straddle the costmap's min/max_obstacle_height window.
        self.declare_parameter("height", 0.10)
        # A peer whose pose is older than this is dropped rather than left as a
        # ghost obstacle blocking the room forever.
        self.declare_parameter("pose_timeout", 2.0)

        self.robots = list(self.get_parameter("robots").value)
        self.radius = float(self.get_parameter("radius").value)
        self.points = int(self.get_parameter("points").value)
        self.height = float(self.get_parameter("height").value)
        self.pose_timeout = float(self.get_parameter("pose_timeout").value)

        self.poses = {}
        for name in self.robots:
            self.create_subscription(
                Odometry, f"/{name}/odom", self._make_cb(name), 10
            )
        self.pubs = {
            name: self.create_publisher(PointCloud2, f"/{name}/peer_points", 10)
            for name in self.robots
        }

        self.create_timer(1.0 / float(self.get_parameter("rate").value), self._publish)
        self.get_logger().info(
            f"sharing peer outlines among {self.robots} (r={self.radius:.2f} m)"
        )

    def _make_cb(self, name):
        def cb(msg):
            self.poses[name] = (
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                self.get_clock().now().nanoseconds * 1e-9,
            )
        return cb

    def _ring(self, cx, cy):
        return [
            (
                cx + self.radius * math.cos(2 * math.pi * i / self.points),
                cy + self.radius * math.sin(2 * math.pi * i / self.points),
                self.height,
            )
            for i in range(self.points)
        ]

    def _publish(self):
        now = self.get_clock().now()
        stale_before = now.nanoseconds * 1e-9 - self.pose_timeout

        header = None
        for name in self.robots:
            pts = []
            for peer, (px, py, stamp) in self.poses.items():
                if peer == name or stamp < stale_before:
                    continue
                pts.extend(self._ring(px, py))

            msg = point_cloud2.create_cloud_xyz32(header or self._header(now), pts)
            header = msg.header
            self.pubs[name].publish(msg)

    def _header(self, now):
        from std_msgs.msg import Header

        h = Header()
        # Poses are world-absolute, and map is the shared frame.
        h.frame_id = "map"
        h.stamp = now.to_msg()
        return h


def main():
    rclpy.init()
    node = PeerObstacles()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
