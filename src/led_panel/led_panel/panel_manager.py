import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
from pathlib import Path
import yaml


class PanelManager(Node):

    def __init__(self):
        super().__init__('panel_manager')

        yaml_path = Path(__file__).resolve().parent.parent / 'config' / 'panels.yaml'
        self.load_config(yaml_path)

        self.get_logger().info(
            f'canvas size is {self.canvas_w} x {self.canvas_h}, '
            f'panel size is {self.panel_w} x {self.panel_h}, '
            f'{len(self.panels)} panels loaded'
        )

        self.panel_publishers = self.create_panel_publishers()

        self.sub = self.create_subscription(Image, 'led/frames_raw', self.on_frame, 10)
        self.get_logger().info('Panel manager ready')

    def load_config(self, yaml_path):
        with open(yaml_path, 'r') as f:
            config = yaml.safe_load(f)

        self.canvas_w = config['canvas']['width']
        self.canvas_h = config['canvas']['height']
        self.panel_w  = config['panel_size']['width']
        self.panel_h  = config['panel_size']['height']
        self.panels   = [p for p in config['panels'] if p['enabled']]

    def create_panel_publishers(self):
        publishers = {}
        for panel in self.panels:
            topic = f"led/panel/{panel['id']}"
            publishers[panel['id']] = self.create_publisher(Image, topic, 10)
        return publishers

    def on_frame(self, msg: Image):
        canvas = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            (msg.height, msg.width, 3)
        )

        for panel in self.panels:
            row, col = panel['row'], panel['col']
            y0 = row * self.panel_h
            y1 = y0 + self.panel_h
            x0 = col * self.panel_w
            x1 = x0 + self.panel_w

            if y1 > self.canvas_h or x1 > self.canvas_w:
                self.get_logger().warning(
                    f"Panel {panel['id']} at ({row},{col}) is out of canvas bounds — skipping"
                )
                continue

            slice_img = canvas[y0:y1, x0:x1]
            self.publish_panel(panel['id'], slice_img, msg.header)
            self.send_to_hardware(panel['id'], slice_img)

    def publish_panel(self, panel_id, pixels, header):
        msg = Image()
        msg.header   = header
        msg.height, msg.width = pixels.shape[:2]
        msg.encoding = 'bgr8'
        msg.step     = msg.width * 3
        msg.data     = pixels.tobytes()
        self.panel_publishers[panel_id].publish(msg)

    def send_to_hardware(self, panel_id, pixels):
        # TODO: implement hardware-specific protocol here
        # pixels is (panel_h, panel_w, 3) uint8, BGR order
        # convert to RGB: pixels[..., ::-1]
        pass


def main():
    rclpy.init()
    node = PanelManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
