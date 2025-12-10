import rclpy
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
        self.second_line_subscriber_ = self.create_subscription(
            String,
            '/lcd_display/second_string',
            self.__second_line_callback,
            10
        )

        self._lcd = LCD(i2c_addr=39)

        self._first_line = ''
        self._second_line = ''

    def __first_line_callback(self, msg: String) -> None:
        self.get_logger().info(f'Received first line {msg.data}')
        self._first_line = msg.data
        self.__refresh_display()
    
    def __second_line_callback(self, msg: String) -> None:
        self.get_logger().info(f'Received second line {msg.data}')
        self._second_line = msg.data
        self.__refresh_display()

    def __refresh_display(self) -> None:
        self._lcd.clear()
        self._lcd.message(self._first_line, 1)
        self._lcd.message(self._second_line, 2)



def main(args=None):
    rclpy.init(args=args)

    node = LCDDisplayNode()

    print('Starting to spin the LCD display node now')
    try:
        node.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
