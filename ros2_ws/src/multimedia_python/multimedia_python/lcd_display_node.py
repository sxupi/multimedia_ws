from rclpy.node import Node
from std_msgs.msg import String

from .external.LCD import LCD

class LCDDisplayNode(Node):

    def __init__(self):
        super().__init__('lcd_displayer')
        self.first_line_subscriber_ = self.create_subscription(
            String,
            '/lcd_display/first_string',
            self.__first_line_callback,
            10
        )

#        self._lcd = LCD(i2c_addr=39)

    def __first_line_callback(self, msg: String) -> None:
        self.get_logger().info(f'Received first line {msg.data}')