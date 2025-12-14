import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32, String


class FrequencyControllerNode(Node):

    COMMAND_FREQ_CHANGE: int = 1
    MIN_FREQUENCY: int = 875
    MAX_FREQUENCY: int = 1090

    def __init__(self):
        super().__init__('frequency_controller')
        #self.command_subscriber_ = self.create_subscription(
        #    String,
        #    '/remote/command_string',
        #    self._frequency_command_callback,
        #    10
        #)
        self._frequency_subscriber_ = self.create_subscription(
            Float32,
            'potentiometer/frequency_float32',
            self._frequency_received_callback,
            10
        )
        self._frequency_publisher_ = self.create_publisher(
            Int32,
            '/current/frequency_int32',
            10
        )
        self._current_frequency = 875

    #def _frequency_command_callback(self, msg: String) -> None:
    #    match msg.data:
    #        case 'NEXT':
    #            self.__higher_frequency()
    #            return
    #        case 'PREV':
    #            self.__lower_frequency()
    #            return
    #        case _:
    #            return

    def _frequency_received_callback(self, msg: Float32) -> None:
        self.get_logger().info('Received frequency: %f' % msg.data)
        # Normalization is required, from percentage to the frequencies
        self._current_frequency = self._map_frequency(msg.data)
        self._publish_frequency()

    def _publish_frequency(self) -> None:
        msg = Int32()
        msg.data = self._current_frequency
        self._frequency_publisher_.publish(msg)
        self.get_logger().info('Publishing frequency: %d' % msg.data)

    def _map_frequency(self, norm_freq: float) -> int:
        return int(self.MIN_FREQUENCY + norm_freq * (self.MAX_FREQUENCY - self.MIN_FREQUENCY))

    def get_frequency(self) -> float:
        return self._current_frequency

    def _higher_frequency(self) -> None:
        self._current_frequency += self.COMMAND_FREQ_CHANGE
        if self._current_frequency > self.MAX_FREQUENCY:
            self._current_frequency = self.MAX_FREQUENCY
        self._publish_frequency()

    def _lower_frequency(self) -> None:
        self._current_frequency -= self.COMMAND_FREQ_CHANGE
        if self._current_frequency < self.MIN_FREQUENCY:
            self._current_frequency = self.MIN_FREQUENCY
        self._publish_frequency()



def main(args=None):
    rclpy.init(args=args)

    node = FrequencyControllerNode()

    print('Starting to spin the frequency controller node now')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
