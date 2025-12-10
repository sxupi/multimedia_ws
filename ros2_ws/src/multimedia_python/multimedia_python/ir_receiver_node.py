from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO

from .external import IRModule


class IRReceiverNode(Node):

    COMMAND_MAPPINGS = {
        16754775: 'PLAY_STOP',  # 0x300ffa857: 'PLAY_STOP',
        16720605: 'SWITCH',  # 0x300ff22dd: 'SWITCH',
        16712445: 'HIGHER_VOLUME',  # 0x300ff02fd: 'HIGHER_VOLUME',
        16750695: 'LOWER_VOLUME',  # 0x300ff9867: 'LOWER_VOLUME',
        16748655: 'NEXT',  # 0x300ff906f: 'NEXT',
        16769055: 'PREV'  # 0x300ffe01f: 'PREV'
    }

    def __init__(self, bcm_num=23):
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
        GPIO.setup(bcm_num, GPIO.IN)
        GPIO.add_event_detect(bcm_num, GPIO.BOTH,
                              callback=self.__ir_module.pWidth)

        # No need to print out high and low durations
        self.__ir_module.set_verbose(False)

        self.get_logger().info(
            'Initialized IR receiver node with commands: {0}'.format(self.COMMAND_MAPPINGS))

    def __ir_received(self, code) -> None:
        
        self.get_logger().info('Command: {0} ({1}) from {2}'.format(
            self.COMMAND_MAPPINGS.get(code), code, self.COMMAND_MAPPINGS))
        if code <= 0:
            return

        converted_code = self.COMMAND_MAPPINGS.get(code)

        if converted_code is not None:
            msg = String()
            msg.data = converted_code
            self.__command_publisher_.publish(msg)
            self.get_logger().info('Publishing received command: {0} ({1})'.format(msg.data, code))
