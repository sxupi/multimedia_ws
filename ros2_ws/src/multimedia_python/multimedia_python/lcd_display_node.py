from rclpy.node import Node
from std_msgs.msg import String

from .external.LCD import LCD

class LCDDisplayNode(Node):

    def __init__(self):
        super().__init__('lcd_displayer')
        self.command_subscriber_ = self.create_subscription(
            String,
            '/display/first_line_string',
            self.__set_first_line_callback,
            10
        )
        self.command_subscriber_ = self.create_subscription(
            String,
            '/display/second_line_string',
            self.__set_second_line_callback,
            10
        )

        self._lcd = LCD(i2c_addr=39)

        self._first_line = 'Hello,'
        self._second_line = 'multimedia station started'
        
        self.__refresh_display()

    def __set_first_line_callback(self, msg: String) -> None:
        self.get_logger().info(f'Received first line data: {msg.data}')
        self._first_line = msg.data
        self.__refresh_display()

    def __set_second_line_callback(self, msg: String) -> None:
        self.get_logger().info(f'Received second line data: {msg.data}')
        self._second_line = msg.data
        self.__refresh_display()

    def __refresh_display(self) -> None:
        self.get_logger().info(f'Refreshing the screen with first line="{self._first_line}" and second line="{self._second_line}"')
        self._lcd.clear()
        self._lcd.message(self._first_line, 1)
        self._lcd.message(self._second_line, 2)