#!/usr/bin/env python3
import re
import subprocess
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

PATTERN = re.compile(r"IR code:\s*([0-9a-fA-F]+)")


class IrReceiverNode(Node):

    def __init__(self):
        super().__init__('ir_receiver_node')

        self.ir_script = self.declare_parameter('ir_script_path',
                                                '/home/filip/ir.py').value

        self.pub = self.create_publisher(String, '/controller/command', 10)

        self.mapping = {
            # Example:
            # "f730cf": "next",
            # "e718ff": "previous",
            # "a15eff": "volume_up",
            # ...
        }

        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.loop, daemon=True)

        try:
            self.proc = subprocess.Popen(
                ["python3", self.ir_script],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1)
        except Exception as e:
            self.get_logger().error(f"Failed to start ir.py: {e}")
            self.proc = None

        self.thread.start()
        self.get_logger().info("IR Receiver Node started.")

    def loop(self):
        if not self.proc or not self.proc.stdout:
            return

        for line in self.proc.stdout:
            if self.stop.is_set():
                break
            m = PATTERN.search(line.strip())
            if not m:
                continue

            code = m.group(1).lower()
            cmd = self.mapping.get(code)
            if not cmd:
                self.get_logger().info(f"Unknown IR code: {code}")
                continue

            self.pub.publish(String(data=cmd))
            self.get_logger().info(f"Published command: {cmd}")

    def destroy_node(self):
        self.stop.set()
        try:
            self.proc.terminate()
        except:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IrReceiverNode()
    rclpy.spin(node)
    rclpy.shutdown()
