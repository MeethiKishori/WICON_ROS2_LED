import rclpy
from rclpy.node import Node 


class led_panel(Node):

    def __init__(self):
        super().__init__('led_panel')

        self.create_timer(1, self.on_timer)


    def on_timer(self):
        self.get_logger().info(f'led panel is creating{self.get_clock().now().to_msg()}')



def main():
    rclpy.init()
    node = led_panel()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    

if __name__=='__main__':
    main()