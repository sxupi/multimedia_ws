import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32, String

from .external.si4703Library import si4703Radio


class RadioNode(Node):

    def __init__(self):
        self.frequency_subscriber_ = self.create_subscription(
            Int32,
            '/current/frequency_int32',
            self.__frequency_change_callback,
            10
        )
        self.volume_subscriber_ = self.create_subscription(
            Int32,
            '/current/volume_float32',
            self.__volume_change_callback,
            10
        )
        self.command_subscriber_ = self.create_subscription(
            String,
            '/remote/command_string',
            self.__incoming_command_callback,
            10
        )
        self.first_line_publisher_ = self.create_publisher(
            String,
            '/lcd_display/first_string',
            10
        )
        self.second_line_publisher_ = self.create_publisher(
            String,
            '/lcd_display/second_string',
            10
        )

        self._radio = si4703Radio(i2cAddr=0x10, resetPin=23)

    def __volume_change_callback(self, msg: Float32) -> None:
        pass

    def __frequency_change_callback(self, msg: Int32) -> None:
        pass

    def __incoming_command_callback(self, msg: String) -> None:
        pass


def main(args=None):
    rclpy.init(args=args)

    node = RadioNode()

    print('Starting to spin the radio node now')
    try:
        node.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
