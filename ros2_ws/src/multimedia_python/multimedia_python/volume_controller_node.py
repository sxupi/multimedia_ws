import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String


class VolumeControllerNode(Node):

    COMMAND_VOLUME_CHANGE: float = 0.05
    MIN_VOLUME: float = 0.0
    MAX_VOLUME: float = 1.0

    def __init__(self):
        super().__init__('volume_controller')
        self._command_subscriber_ = self.create_subscription(
            String,
            '/remote/command_string',
            self._volume_command_callback,
            10
        )
        self._volume_pot_subscriber_ = self.create_subscription(
            Float32,
            '/potentiometer/volume_float32',
            self._volume_received_callback,
            10
        )
        self._volume_web_subscriber_ = self.create_subscription(
            Float32,
            '/web_remote/volume_float32',
            self._volume_received_callback,
            10
        )
        self._volume_publisher_ = self.create_publisher(
            Float32,
            '/current/volume_float32',
            10
        )
        self._current_volume = 0

    def _volume_command_callback(self, msg: String) -> None:
        match msg.data:
            case 'HIGHER_VOLUME':
                self._higher_volume()
                return
            case 'LOWER_VOLUME':
                self._lower_volume()
                return
            case _:
                return

    def _volume_received_callback(self, msg: Float32) -> None:
        self.get_logger().info('Received volume: %f' % msg.data)
        # No normalization is required here
        self._current_volume = round(msg.data, 2)
        self._publish_volume()

    def _publish_volume(self) -> None:
        msg = Float32()
        msg.data = self._current_volume
        self._volume_publisher_.publish(msg)

        self.get_logger().info('Publishing current volume: %f' % msg.data)

    def get_volume(self) -> float:
        return self._current_volume

    def _higher_volume(self) -> None:
        self._current_volume += self.COMMAND_VOLUME_CHANGE
        if self._current_volume > self.MAX_VOLUME:
            self._current_volume = self.MAX_VOLUME
        self._publish_volume()

    def _lower_volume(self) -> None:
        self._current_volume -= self.COMMAND_VOLUME_CHANGE
        if self._current_volume < self.MIN_VOLUME:
            self._current_volume = self.MIN_VOLUME
        self._publish_volume()


def main(args=None):
    rclpy.init(args=args)

    node = VolumeControllerNode()

    print('Starting to spin the volume controller node now')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
