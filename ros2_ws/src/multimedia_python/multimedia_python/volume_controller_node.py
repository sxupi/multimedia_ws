from rclpy.node import Node
from std_msgs.msg import Float32, String


class VolumeControllerNode(Node):

    COMMAND_VOLUME_CHANGE: float = 0.05
    MIN_VOLUME: float = 0.0
    MAX_VOLUME: float = 1.0

    def __init__(self):
        super().__init__('volume_controller')
        self.command_subscriber_ = self.create_subscription(
            String,
            '/ir_receiver/command_string',
            self.__volume_command_callback,
            10
        )
        self.volume_subscriber_ = self.create_subscription(
            Float32,
            '/potentiometer/volume_float32',
            self.__volume_received_callback,
            10
        )
        self.volume_publisher_ = self.create_publisher(
            Float32,
            '/current/volume_float32',
            10
        )
        self.__current_volume = 0

    def __volume_command_callback(self, msg: String) -> None:
        match msg.data:
            case 'HIGHER_VOLUME':
                self.__higher_volume()
                return
            case 'LOWER_VOLUME':
                self.__lower_volume()
                return
            case _:
                return

    def __volume_received_callback(self, msg: Float32) -> None:
        self.get_logger().info('Received volume: %f' % msg.data)
        # No normalization is required here
        self.__current_volume = round(msg.data, 2)
        self.__publish_volume()

    def __publish_volume(self) -> None:
        msg = Float32()
        msg.data = self.__current_volume
        self.volume_publisher_.publish(msg)
        self.get_logger().info('Publishing current volume: %f' % msg.data)

    def get_volume(self) -> float:
        return self.__current_volume

    def __higher_volume(self) -> None:
        self.__current_volume += self.COMMAND_VOLUME_CHANGE
        if self.__current_volume > self.MAX_VOLUME:
            self.__current_volume = self.MAX_VOLUME
        self.__publish_volume()

    def __lower_volume(self) -> None:
        self.__current_volume -= self.COMMAND_VOLUME_CHANGE
        if self.__current_volume < self.MIN_VOLUME:
            self.__current_volume = self.MIN_VOLUME
        self.__publish_volume()
