import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid, Path
import cv2
import numpy as np


class ShowPath(Node):

    def __init__(self):
        super().__init__('show_path')

        self.map_img  = None
        self.map_info = None
        self.path_pts = []

        self.create_subscription(OccupancyGrid, '/map',  self.on_map,  1)
        self.create_subscription(Path,          '/plan', self.on_path, 10)
        self.pub = self.create_publisher(Image, 'led/frames_raw', 10)
        self.create_timer(0.05, self.publish_frame)

        cv2.namedWindow('Map + Path')
        self.get_logger().info('ShowPath ready')

    def on_map(self, msg: OccupancyGrid):
        self.map_info = msg.info
        grid = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))

        bgr = np.full((msg.info.height, msg.info.width, 3), 80, dtype=np.uint8)
        bgr[grid == 0]   = (255, 255, 255)
        bgr[grid == 100] = (0, 0, 0)

        self.map_img = cv2.flip(bgr, 0)

    def on_path(self, msg: Path):
        self.path_pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def world_to_px(self, x, y):
        info = self.map_info
        col = int((x - info.origin.position.x) / info.resolution)
        row = info.height - 1 - int((y - info.origin.position.y) / info.resolution)
        return col, row

    def publish_frame(self):
        if self.map_img is None:
            return

        frame = self.map_img.copy()
        h, w  = frame.shape[:2]

        if len(self.path_pts) > 1:
            pts = [(c, r) for c, r in (self.world_to_px(x, y) for x, y in self.path_pts)
                   if 0 <= c < w and 0 <= r < h]
            for i in range(len(pts) - 1):
                cv2.line(frame, pts[i], pts[i + 1], (0, 220, 80), 2)
            if pts:
                cv2.circle(frame, pts[0],  3, (255, 0, 0), -1)
                cv2.circle(frame, pts[-1], 4, (0, 0, 255), -1)

        msg = Image()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'led_panel'
        msg.height, msg.width = frame.shape[:2]
        msg.encoding = 'bgr8'
        msg.step     = msg.width * 3
        msg.data     = frame.tobytes()
        self.pub.publish(msg)

        cv2.imshow('Map + Path', frame)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = ShowPath()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
