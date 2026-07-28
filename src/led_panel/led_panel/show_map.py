import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from ament_index_python.packages import get_package_share_directory
import cv2
import numpy as np
import yaml
import os
import sys
from pathlib import Path


# Saved maps live in the source tree; map_saver_cli writes there, not into share/
DEFAULT_MAP_DIR = Path(__file__).resolve().parents[2] / 'nav2_mobile_robot' / 'map'
DEFAULT_MAP = 'map_led2.pgm'


def resolve_map_path(name: str) -> str:
    """Accept a full path, or a bare map name looked up in the map dir."""
    p = Path(name)
    if p.suffix != '.pgm':
        p = p.with_suffix('.pgm')
    if p.is_absolute() or p.exists():
        return str(p)
    return str(DEFAULT_MAP_DIR / p.name)

# PGM occupancy thresholds (from map_led.yaml)
OCCUPIED_THRESH = int(254 * (1 - 0.65))   # <= 89  → occupied
FREE_THRESH     = int(254 * (1 - 0.25))   # >= 191 → free


def load_canvas_size():
    config_path = Path(__file__).resolve().parent.parent / 'config' / 'panels.yaml'
    with open(config_path, 'r') as f:
        cfg = yaml.safe_load(f)
    return cfg['canvas']['width'], cfg['canvas']['height']


def pgm_to_bgr(pgm: np.ndarray) -> np.ndarray:
    bgr = np.zeros((*pgm.shape, 3), dtype=np.uint8)
    free     = pgm >= FREE_THRESH
    occupied = pgm <= OCCUPIED_THRESH
    unknown  = ~free & ~occupied

    bgr[free]     = (255, 255, 255)
    bgr[occupied] = (0,   0,   0)
    bgr[unknown]  = (80,  80,  80)
    return bgr


class ShowMap(Node):

    def __init__(self, map_path):
        super().__init__('show_map')

        self.map_path = map_path
        self.canvas_w, self.canvas_h = load_canvas_size()

        raw = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)
        if raw is None:
            self.get_logger().error(f'Cannot load map: {map_path}')
            raise FileNotFoundError(map_path)

        resized = cv2.resize(raw, (self.canvas_w, self.canvas_h), interpolation=cv2.INTER_NEAREST)
        self.frame = pgm_to_bgr(resized)

        self.pub = self.create_publisher(Image, 'led/frames_raw', 10)
        self.create_timer(0.05, self.publish_frame)

        cv2.namedWindow('Map on LED Panel')
        self.get_logger().info(
            f'Loaded {map_path}  original {raw.shape} → {self.canvas_w}x{self.canvas_h}'
        )

    def publish_frame(self):
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'led_panel'
        msg.width    = self.canvas_w
        msg.height   = self.canvas_h
        msg.encoding = 'bgr8'
        msg.step     = self.canvas_w * 3
        msg.data     = self.frame.tobytes()
        self.pub.publish(msg)

        cv2.imshow('Map on LED Panel', self.frame)
        cv2.waitKey(1)


def main(argv=None):
    argv = rclpy.utilities.remove_ros_args(sys.argv if argv is None else argv)
    map_path = resolve_map_path(argv[1] if len(argv) > 1 else DEFAULT_MAP)

    rclpy.init()
    node = ShowMap(map_path)
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
