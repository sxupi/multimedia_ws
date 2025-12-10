from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO

from .external import IRModule


class IRReceiverNode(Node):

    COMMAND_MAPPINGS = {
        0x300ffa857: 'PLAY_STOP',
        0x300ff22dd: 'SWITCH',
        0x300ff02fd: 'HIGHER_VOLUME',
        0x300ff9867: 'LOWER_VOLUME',
        0x300ff906f: 'NEXT',
        0x300ffe01f: 'PREV'
    }

    def __init__(self, pin=16):
        super().__init__('ir_receiver')
        self.__command_publisher_ = self.create_publisher(
            String,
            '/ir_receiver/command',
            10
        )
        self.__ir_module = IRModule.IRRemote(self.__ir_received)

        # Needs to be set
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(pin, GPIO.IN)
        GPIO.add_event_detect(pin, GPIO.BOTH, callback=self.__ir_module.pWidth)

        # No need to print out high and low durations
        self.__ir_module.set_verbose(False)

    def __ir_received(self, code) -> None:
        msg = String()
        self.get_logger().info('Command: {0} from {1}'.format(code, self.COMMAND_MAPPINGS))
        msg.data = self.COMMAND_MAPPINGS[code]
        self.__command_publisher_.publish(msg)
        self.get_logger().info('Publishing received command: {0} ({1})'.format(msg.data, code))
