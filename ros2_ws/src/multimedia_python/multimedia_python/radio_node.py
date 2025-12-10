import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32, String

from .external.si4703Library import si4703Radio


class RadioNode(Node):

    MIN_VOLUME = 0
    MAX_VOLUME = 15

    def __init__(self):
        super().__init__('radio')
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
        self.frequency_publisher_ = self.create_publisher(
            Int32,
            '/current/frequency_int32',
            10,
        )
        self.first_line_publisher_ = self.create_publisher(
            String,
            '/display/header_string',
            10
        )
        self.second_line_publisher_ = self.create_publisher(
            String,
            '/display/text_string',
            10
        )

        self._radio = si4703Radio(i2cAddr=0x10, resetPin=23)

        self._curr_freq = 900
        self._curr_volume = 0.0

    def __volume_change_callback(self, msg: Float32) -> None:
        converted_volume = self.__convert_volume(msg.data)
        self._radio.si4703SetVolume(converted_volume)
        self.get_logger().info(f'Set volume to {converted_volume}')

    def __convert_volume(self, volume: float) -> int:
        return int(self.MIN_VOLUME + volume * (self.MAX_VOLUME - self.MIN_VOLUME))

    def __frequency_change_callback(self, msg: Int32) -> None:
        if self._curr_freq != msg.data:
            self._radio.si4703SetChannel(msg.data)
            self.get_logger().info(f'Set channel to frequency {msg.data}')
            self.__switch_channels(False)
        else:
            self.get_logger().info(f'Frequency stayed the same {msg.data}')

    def __incoming_command_callback(self, msg: String) -> None:
        match msg.data:
            case 'PLAY_STOP':
                self.__play_stop_handle()
                return
            case 'SWITCH':
                self.__switch_handle()
                return
            case 'NEXT':
                self.__find_next_channel()
                return
            case 'PREV':
                self.__find_prev_channel()
                return
            case _:
                return

    def __play_stop_handle(self) -> None:
        # TODO: Implement play/stop
        pass

    def __switch_handle(self) -> None:
        # TODO: Implement switching, but not sure if it is done here
        pass

    def __find_next_channel(self) -> None:
        self._radio.si4703SeekUp()
        self._curr_freq = self._radio.si4703GetChannel()
        self.__switch_channels(True)

    def __find_prev_channel(self) -> None:
        self._radio.si4703SeekDown()
        self._curr_freq = self._radio.si4703GetChannel()
        self.__switch_channels(True)

    def __switch_channels(self, publish_freq: bool) -> None:
        if publish_freq:
            freq_msg = Int32()
            freq_msg.data = self._curr_freq
            self.frequency_publisher_(freq_msg)
            self.get_logger().info(f'Published new frequency {freq_msg.data}')
        self._radio.si4703ClearRDSBuffers()
        self._radio.si4703ProcessRDS()

        program_service_text = self._radio.si4703GetProgramService()
        first_line_msg = String()

        if not program_service_text:
            first_line_msg.data = 'Radio - No station'
        else:
            first_line_msg.data = 'Radio - ' + program_service_text

        self.first_line_publisher_.publish(first_line_msg)
        self.get_logger().info(
            f'Published first line string: {first_line_msg.data}')

        radio_text = self._radio.si4703GetProgramService()
        second_line_msg = String()

        if not program_service_text:
            second_line_msg.data = 'No text'
        else:
            second_line_msg.data = radio_text

        self.first_line_publisher_.publish(second_line_msg)
        self.get_logger().info(
            f'Published second line string: {second_line_msg.data}')


def main(args=None):
    rclpy.init(args=args)

    node = RadioNode()

    print('Starting to spin the radio node now')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
