import yaml
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np

#width = 640
#height = 640

scale = 10



class drawpath_cv(Node):
    
    def __init__(self):
        super().__init__('drawpath_cv')
        self.moved = False
        self.last_x, self.last_y = 0, 0
        
        config_path = Path(__file__).resolve().parent.parent / 'config' / 'panels.yaml'
        self.load_config(config_path)

        self.create_timer(0.02,self.publish_frames)
        self.publish_led=self.create_publisher(Image,'led/frames_raw',10)

        self.get_logger().info('Drawpath running on CV2 window')

        cv2.namedWindow('Draw Path')
        
        cv2.setMouseCallback('Draw Path', self.mouse_moved)

        self.canvas = np.zeros((self.canvas_h, self.canvas_w, 3), dtype=np.uint8)

    def load_config(self, path):
        with open(path, 'r') as f:
            cfg = yaml.safe_load(f)

        
        self.canvas_w = cfg['canvas']['width']
        self.canvas_h = cfg['canvas']['height']


    def publish_frames(self):

         msg = Image()

         msg.header.stamp = self.get_clock().now().to_msg()
         msg.header.frame_id = 'CV window'
         msg.width = self.canvas_w
         msg.height = self.canvas_h
         msg.encoding = 'bgr8'
         msg.step = self.canvas_w * 3
         msg.data = self.canvas.tobytes()
         self.publish_led.publish(msg)

         cv2.imshow('Draw Path', self.canvas)
         cv2.waitKey(1)
    



    def mouse_moved(self,event,x,y,flags,param):


        if event == cv2.EVENT_LBUTTONDOWN:
                self.moved = True
                self.last_x, self.last_y = x, y

        elif event == cv2.EVENT_MOUSEMOVE and self.moved:
                cv2.line(self.canvas, (self.last_x, self.last_y), (x,y), (0,200,133), 5)
                self.last_x, self.last_y = x, y
        
        elif event == cv2.EVENT_LBUTTONUP:
              self.moved = False


def main():
    rclpy.init()
    node = drawpath_cv()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
