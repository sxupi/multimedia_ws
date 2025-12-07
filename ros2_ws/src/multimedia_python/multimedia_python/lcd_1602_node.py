#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from LCD import LCD   # LCD-1602-I2C library


class Lcd1602Node(Node):

    def __init__(self):
        super().__init__('lcd_1602_node')

        self.cols = self.declare_parameter('cols', 16).value
        self.rows = self.declare_parameter('rows', 2).value
        self.addr = self.declare_parameter('i2c_address', 0x3F).value
        self.rev = self.declare_parameter('rpi_revision', 2).value

        self.lcd = LCD(self.rev, self.addr, True)

        self.full1 = ""
        self.full2 = ""
        self.off1 = 0
        self.off2 = 0

        self.create_subscription(
            String, '/1602_display/first_line', self.cb1, 10)
        self.create_subscription(
            String, '/1602_display/second_line', self.cb2, 10)

        self.timer = self.create_timer(0.3, self.update)
        self.get_logger().info("LCD Node started.")

    def cb1(self, msg):
        self.full1 = msg.data
        self.off1 = 0

    def cb2(self, msg):
        self.full2 = msg.data
        self.off2 = 0

    def marquee(self, text, off):
        if len(text) <= self.cols:
            return text.ljust(self.cols), 0
        scroll = text + "   "
        visible = "".join(scroll[(off+i) % len(scroll)]
                          for i in range(self.cols))
        return visible, (off + 1) % len(scroll)

    def update(self):
        l1, self.off1 = self.marquee(self.full1, self.off1)
        l2, self.off2 = self.marquee(self.full2, self.off2)

        try:
            self.lcd.message(l1, 1)
            if self.rows > 1:
                self.lcd.message(l2, 2)
        except Exception as e:
            self.get_logger().warn(f"LCD error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = Lcd1602Node()
    rclpy.spin(node)
    rclpy.shutdown()
