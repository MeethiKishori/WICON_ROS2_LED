import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


class PeerObstacles(Node):
    def __init__(self):
        super().__init__("peer_obstacles")

        self.declare_parameter("robots", ["r1", "r2"])
        self.declare_parameter("radius", 0.16)
        self.declare_parameter("points", 24)
        self.declare_parameter("rate", 10.0)
        self.declare_parameter("height", 0.10)
        self.declare_parameter("pose_timeout", 2.0)

        self.robots = list(self.get_parameter("robots").value)
        self.radius = float(self.get_parameter("radius").value)
        self.points = int(self.get_parameter("points").value)
        self.height = float(self.get_parameter("height").value)
        self.pose_timeout = float(self.get_parameter("pose_timeout").value)
        rate = float(self.get_parameter("rate").value)

        self.poses = {}
        self.cloud_publishers = {}

        for name in self.robots:
            self.create_subscription(Odometry, f"/{name}/odom", self.make_odom_callback(name), 10)
            self.cloud_publishers[name] = self.create_publisher(
                PointCloud2, f"/{name}/peer_points", 10
            )

        self.create_timer(1.0 / rate, self.publish_peers)
        self.get_logger().info(
            f"sharing peer outlines among {self.robots} (r={self.radius:.2f} m)"
        )

    def make_odom_callback(self, name):
        def on_odom(msg):
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            seen_at = self.get_clock().now().nanoseconds * 1e-9
            self.poses[name] = (x, y, seen_at)

        return on_odom

    def circle_points(self, center_x, center_y):
        points = []
        for i in range(self.points):
            angle = 2 * math.pi * i / self.points
            x = center_x + self.radius * math.cos(angle)
            y = center_y + self.radius * math.sin(angle)
            points.append((x, y, self.height))
        return points

    def make_header(self, now):
        header = Header()
        header.frame_id = "map"
        header.stamp = now.to_msg()
        return header

    def publish_peers(self):
        now = self.get_clock().now()
        too_old = now.nanoseconds * 1e-9 - self.pose_timeout
        header = self.make_header(now)

        for name in self.robots:
            points = []
            for peer_name, (peer_x, peer_y, seen_at) in self.poses.items():
                if peer_name == name:
                    continue
                if seen_at < too_old:
                    continue
                points.extend(self.circle_points(peer_x, peer_y))

            msg = point_cloud2.create_cloud_xyz32(header, points)
            self.cloud_publishers[name].publish(msg)


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
