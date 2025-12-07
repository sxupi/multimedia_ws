#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, String, Int32


class ControllNode(Node):
    """
    Central controller for the multimedia station.
    Handles:
    - Potentiometers (volume + frequency)
    - Commands from IR receiver or other sources
    - Publishes final "current" frequency + volume
    - Publishes LCD text lines
    """

    def __init__(self):
        super().__init__('controll_node')

        # Parameters
        self.min_freq_mhz = float(
            self.declare_parameter('min_freq_mhz', 87.5).value)
        self.max_freq_mhz = float(
            self.declare_parameter('max_freq_mhz', 108.0).value)
        self.ir_step_mhz = float(
            self.declare_parameter('ir_step_mhz', 0.1).value)
        self.volume_step = float(
            self.declare_parameter('volume_step', 0.05).value)

        # Internal state
        self.current_frequency_norm = 0.5
        self.current_volume_norm = 0.5

        # Publishers
        self.pub_freq = self.create_publisher(
            Int32, '/current/frequency_int', 10)
        self.pub_vol = self.create_publisher(
            Float32, '/current/volume_float32', 10)
        self.pub_l1 = self.create_publisher(
            String, '/1602_display/first_line', 10)
        self.pub_l2 = self.create_publisher(
            String, '/1602_display/second_line', 10)

        # Subscriptions
        self.create_subscription(Float32, '/potentiometer/frequency_float32',
                                 self.cb_pot_freq, 10)
        self.create_subscription(Float32, '/potentiometer/volume_float32',
                                 self.cb_pot_vol, 10)
        self.create_subscription(String, '/controller/command',
                                 self.cb_command, 10)

        # Timer
        self.timer = self.create_timer(0.2, self.publish_state)

        self.get_logger().info("ControllNode started.")

    # ---------------- internal utility functions ----------------

    def clamp(self):
        self.current_frequency_norm = max(
            0.0, min(1.0, self.current_frequency_norm))
        self.current_volume_norm = max(0.0, min(1.0, self.current_volume_norm))

    def norm_to_mhz(self, n):
        return self.min_freq_mhz + n * (self.max_freq_mhz - self.min_freq_mhz)

    def mhz_to_norm(self, mhz):
        span = (self.max_freq_mhz - self.min_freq_mhz)
        return max(0.0, min(1.0, (mhz - self.min_freq_mhz) / span))

    def mhz_to_int10(self, mhz):
        return int(mhz * 10 + 0.5)

    # ---------------- callbacks ----------------

    def cb_pot_freq(self, msg):
        self.current_frequency_norm = float(msg.data)
        self.clamp()
        self.publish_state()

    def cb_pot_vol(self, msg):
        self.current_volume_norm = float(msg.data)
        self.clamp()
        self.publish_state()

    def cb_command(self, msg):
        cmd = msg.data.lower().strip()

        if cmd == "next":
            f = self.norm_to_mhz(
                self.current_frequency_norm) + self.ir_step_mhz
            self.current_frequency_norm = self.mhz_to_norm(f)

        elif cmd == "previous":
            f = self.norm_to_mhz(
                self.current_frequency_norm) - self.ir_step_mhz
            self.current_frequency_norm = self.mhz_to_norm(f)

        elif cmd == "volume_up":
            self.current_volume_norm += self.volume_step

        elif cmd == "volume_down":
            self.current_volume_norm -= self.volume_step

        else:
            self.get_logger().warn(f"Unknown command: {cmd}")
            return

        self.clamp()
        self.publish_state()

    # ---------------- publisher ----------------

    def publish_state(self):
        freq_mhz = self.norm_to_mhz(self.current_frequency_norm)
        freq_int = self.mhz_to_int10(freq_mhz)

        self.pub_freq.publish(Int32(data=freq_int))
        self.pub_vol.publish(Float32(data=self.current_volume_norm))

        # update LCD lines
        self.pub_l1.publish(String(data=f"Freq: {freq_mhz:5.1f} MHz"))
        self.pub_l2.publish(
            String(data=f"Vol:  {self.current_volume_norm:4.2f}"))


def main(args=None):
    rclpy.init(args=args)
    node = ControllNode()
    rclpy.spin(node)
    rclpy.shutdown()
