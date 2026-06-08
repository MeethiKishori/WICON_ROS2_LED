import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import yaml
import os


class PanelManager(Node):

    def __init__(self):
        super().__init__('panel_manager')

        config_path = os.path.join(os.path.dirname(__file__), '..', '..', 'config', 'panels.yaml')

        self.load_config(config_path)
        self.panel_publishers = self.create_panel_publishers()

        self.create_subscription(Image, 'led/frames_raw', self.on_frame, 10)
        self.get_logger().info(f'Panel manager ready — {len(self.panels)} panels loaded')

    def load_config(self, path):
        with open(path, 'r') as f:
            cfg = yaml.safe_load(f)

        self.canvas_w = cfg['canvas']['width']
        self.canvas_h = cfg['canvas']['height']
        self.panel_w = cfg['panel_size']['width']
        self.panel_h = cfg['panel_size']['height']
        self.panels = [p for p in cfg['panels'] if p.get('enabled', True)]

        self.get_logger().info(
            f'Canvas: {self.canvas_w}x{self.canvas_h}, '
            f'Panel size: {self.panel_w}x{self.panel_h}, '
            f'Active panels: {len(self.panels)}'
        )

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
                self.get_logger().warn(
                    f"Panel {panel['id']} at ({row},{col}) is out of canvas bounds — skipping"
                )
                continue

            slice_img = canvas[y0:y1, x0:x1]
            self.publish_panel(panel['id'], slice_img, msg.header)

            # --- stub: replace this with actual hardware send ---
            self.send_to_hardware(panel['id'], slice_img)

    def publish_panel(self, panel_id, pixels, header):
        msg = Image()
        msg.header = header
        msg.height, msg.width = pixels.shape[:2]
        msg.encoding = 'bgr8'
        msg.step = msg.width * 3
        msg.data = pixels.tobytes()
        self.panel_publishers[panel_id].publish(msg)

    def send_to_hardware(self, panel_id, pixels):
        # TODO: implement hardware-specific protocol here
        # pixels is a numpy array of shape (panel_h, panel_w, 3) dtype uint8
        # RGB order: pixels[..., ::-1] to convert BGR -> RGB if needed
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
