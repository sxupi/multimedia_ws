import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .utils.tft_two_line_display import create_device, TwoLineDisplay


class DisplayNode(Node):

    def __init__(self):
        super().__init__('display')
        self.first_line_subscriber_ = self.create_subscription(
            String,
            '/display/header_string',
            self.__header_callback,
            10
        )
        self.second_line_subscriber_ = self.create_subscription(
            String,
            '/display/text_string',
            self.__text_callback,
            10
        )

        self._device = create_device()
        self._display = TwoLineDisplay(self._device)

        self._timer = self.create_timer(0.02, self.__refresh_display)

        self._display.display_header('Waiting...')
        self._display.display_text('To receive some text...')

    def __header_callback(self, msg: String) -> None:
        self.get_logger().info(f'Received first line {msg.data}')
        self._display.display_header(msg.data)

    def __text_callback(self, msg: String) -> None:
        self.get_logger().info(f'Received second line {msg.data}')
        self._display.display_text(msg.data)

    def __refresh_display(self) -> None:
        self._display.update()


def main(args=None):
    rclpy.init(args=args)

    node = DisplayNode()

    print('Starting to spin the LCD display node now')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
