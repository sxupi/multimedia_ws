#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32

from si4703Library import si4703Radio


class Si470xNode(Node):

    def __init__(self):
        super().__init__('si470x_node')

        addr = self.declare_parameter('i2c_address', 0x10).value
        rst = self.declare_parameter('reset_pin', 5).value
        irq = self.declare_parameter('irq_pin', -1).value

        if irq == -1:
            self.radio = si4703Radio(addr, rst)
        else:
            self.radio = si4703Radio(addr, rst, irq)

        self.radio.si4703Init()

        self.create_subscription(
            Int32,   '/current/frequency_int',   self.cb_freq, 10)
        self.create_subscription(
            Float32, '/current/volume_float32',  self.cb_vol,  10)

        self.get_logger().info("Si470x Node started.")

    def cb_freq(self, msg):
        ch = int(msg.data)
        try:
            self.radio.si4703SetChannel(ch)
            self.get_logger().info(f"Set freq -> {ch/10:.1f} MHz")
        except Exception as e:
            self.get_logger().error(str(e))

    def cb_vol(self, msg):
        vol_norm = max(0.0, min(1.0, float(msg.data)))
        hw = int(round(vol_norm * 15))
        try:
            self.radio.si4703SetVolume(hw)
            self.get_logger().info(f"Set volume -> {hw}")
        except Exception as e:
            self.get_logger().error(str(e))


def main(args=None):
    rclpy.init(args=args)
    node = Si470xNode()
    rclpy.spin(node)
    rclpy.shutdown()
