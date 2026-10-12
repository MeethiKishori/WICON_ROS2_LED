import os
import sys

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import Image


# Folder of this file: .../src/led_panel/led_panel
THIS_DIR = os.path.dirname(os.path.abspath(__file__))

# Saved maps live in the source tree, next to the led_panel package
MAP_DIR = os.path.join(THIS_DIR, '..', '..', 'nav2_mobile_robot', 'map')
MAP_FILE = 'map_led2.pgm'

CONFIG_FILE = os.path.join(THIS_DIR, '..', 'config', 'panels.yaml')

# PGM occupancy thresholds (from map_led2.yaml) since yaml threshold is on occupancy probability,
#its occupied if pixel_value < 255 - occ_thr*255, its wall if pixel <= 89
#its free if pixel_value > 255 - free_thr*255, its free if pixel >= 206
OCCUPIED_THRESH = 89    #  occupied if pixel_value < 255 - 0.65*255 = 89.25
FREE_THRESH = 206       #  free if pixel_value > 255 - 0.196*255 = 205.02


def load_canvas_size():
    """Read the LED canvas width and height from panels.yaml."""
    f = open(CONFIG_FILE, 'r')
    config = yaml.safe_load(f)
    f.close()

    width = config['canvas']['width']
    height = config['canvas']['height']
    return width, height


def pgm_to_bgr(pgm):
    """Turn a grey map image into a black / white / grey colour image."""
    height = pgm.shape[0]
    width = pgm.shape[1]
    bgr = np.zeros((height, width, 3), dtype=np.uint8) # 3 channels for BGR

    for y in range(height):
        for x in range(width):
            value = pgm[y][x]
            if value >= FREE_THRESH:
                color = (255, 255, 255)      # free space -> white
            elif value <= OCCUPIED_THRESH:
                color = (0, 0, 0)            # wall -> black
            else:
                color = (208, 244, 64)         # unknown -> grey
            bgr[y][x] = color

    return bgr


class ShowMap(Node):

    def __init__(self, map_path):
        super().__init__('show_map')

        self.width, self.height = load_canvas_size()

        # Load the map as a grey image and fit it to the LED canvas
        raw = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)
        if raw is None:
            print('Cannot load map:', map_path)
            sys.exit(1)

        small = cv2.resize(raw, (self.width, self.height))
        self.frame = pgm_to_bgr(small)

        # Send the same picture 20 times per second
        self.pub = self.create_publisher(Image, 'led/frames_raw', 10)
        self.create_timer(0.05, self.publish_frame)

        print('Loaded', map_path, '->', self.width, 'x', self.height)

    def publish_frame(self):
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'led_panel'
        msg.width = self.width
        msg.height = self.height
        msg.encoding = 'bgr8'
        msg.step = self.width * 3
        msg.data = self.frame.tobytes()
        self.pub.publish(msg)

        cv2.imshow('Map on LED Panel', self.frame)
        cv2.waitKey(1)


def main():
    # Use the map name given on the command line, else the default one
    args = rclpy.utilities.remove_ros_args(sys.argv)
    if len(args) > 1:
        map_name = args[1]
    else:
        map_name = MAP_FILE
    map_path = os.path.join(MAP_DIR, map_name)

    rclpy.init()
    node = ShowMap(map_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
