import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from ament_index_python.packages import get_package_share_directory
import cv2
import numpy as np
import os


MAP_PATH = os.path.join(
    get_package_share_directory('nav2_mobile_robot'), 'map_led2.pgm'
)

PANEL_SIZE = 640

# PGM occupancy thresholds (from map_led.yaml)
OCCUPIED_THRESH = int(254 * (1 - 0.65))   # <= 89  → occupied
FREE_THRESH     = int(254 * (1 - 0.25))   # >= 191 → free


def pgm_to_bgr(pgm: np.ndarray) -> np.ndarray:
    """Convert grayscale occupancy PGM to a colourised BGR image."""
    bgr = np.zeros((*pgm.shape, 3), dtype=np.uint8)
    free     = pgm >= FREE_THRESH
    occupied = pgm <= OCCUPIED_THRESH
    unknown  = ~free & ~occupied

    bgr[free]     = (255, 255, 255)   # white  – free space
    bgr[occupied] = (0,   0,   0)     # black  – walls/obstacles
    bgr[unknown]  = (80,  80,  80)    # grey   – unknown
    return bgr


class ShowMap(Node):

    def __init__(self):
        super().__init__('show_map')

        raw = cv2.imread(MAP_PATH, cv2.IMREAD_GRAYSCALE)
        if raw is None:
            self.get_logger().error(f'Cannot load map: {MAP_PATH}')
            raise FileNotFoundError(MAP_PATH)

        resized = cv2.resize(raw, (PANEL_SIZE, PANEL_SIZE), interpolation=cv2.INTER_NEAREST)
        self.frame = pgm_to_bgr(resized)

        self.pub = self.create_publisher(Image, 'led/frames_raw', 10)
        self.create_timer(0.05, self.publish_frame)

        cv2.namedWindow('Map on LED Panel')
        self.get_logger().info(f'Loaded {MAP_PATH}  original {raw.shape} → {PANEL_SIZE}x{PANEL_SIZE}')

    def publish_frame(self):
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'led_panel'
        msg.width  = PANEL_SIZE
        msg.height = PANEL_SIZE
        msg.encoding = 'bgr8'
        msg.step = PANEL_SIZE * 3
        msg.data = self.frame.tobytes()
        self.pub.publish(msg)

        cv2.imshow('Map on LED Panel', self.frame)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = ShowMap()
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
