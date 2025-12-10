from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO
import queue

from .external import IRModule


class IRReceiverNode(Node):

    COMMAND_MAPPINGS = {
        0x300ffa857: 'PLAY_STOP',
        0x300ff22dd: 'SWITCH',
        0x300ff02fd: 'HIGHER_VOLUME',
        0x300ff9867: 'LOWER_VOLUME',
        0x300ff906f: 'NEXT',
        0x300ffe01f: 'PREV',
    }

    def __init__(self, pin: int = 23) -> None:
        super().__init__('ir_receiver')

        self._pin = pin
        self._command_pub = self.create_publisher(
            String,
            '/ir_receiver/command',
            10
        )

        # Thread-safe queue: external threads push, ROS timer pops
        self._code_queue: queue.Queue[int] = queue.Queue(maxsize=100)

        # IR decoder: will call _on_ir_code() from its own thread
        self._ir = IRModule.IRRemote(self._on_ir_code)

        # GPIO setup (BCM numbering)
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self._pin, GPIO.IN)
        GPIO.add_event_detect(self._pin, GPIO.BOTH, callback=self._ir.pWidth)

        # IR module config: no verbose timings, repeat enabled like example
        self._ir.set_verbose(False)
        self._ir.set_repeat(True)

        # Timer to process queued codes in ROS2 context
        # 0.02 s = 50 Hz; adjust as you like
        self._timer = self.create_timer(0.02, self._process_codes)

        self.get_logger().info(
            f"Initialized IR receiver node on GPIO {self._pin} with commands: {self.COMMAND_MAPPINGS}"
        )

    # ---------- EXTERNAL THREAD SIDE ----------

    def _on_ir_code(self, code: int) -> None:
        """
        Called by IRModule.IRRemote from its own thread.
        Must NOT touch ROS directly; just enqueue.
        """
        try:
            self._code_queue.put_nowait(code)
            print(code)
        except queue.Full:
            # Option A: drop the new code
            # self.get_logger() is NOT safe here (external thread),
            # so avoid logging; just drop silently.
            try:
                # Option B: drop oldest and insert newest
                _ = self._code_queue.get_nowait()
                self._code_queue.put_nowait(code)
            except queue.Empty:
                pass

    # ---------- ROS EXECUTOR SIDE ----------

    def _process_codes(self) -> None:
        """
        Runs in ROS2 executor thread via timer.
        Safe place to use publishers and logging.
        """
        processed_any = False

        while not self._code_queue.empty():
            processed_any = True
            try:
                code = self._code_queue.get_nowait()
            except queue.Empty:
                break

            self.get_logger().debug(f"Processing IR code from queue: {code}")

            command = self.COMMAND_MAPPINGS.get(code)

            msg = String()
            msg.data = command
            self._command_pub.publish(msg)
            self.get_logger().info(
                f"Publishing received command: {msg.data} ({code})"
            )

    # ---------- CLEANUP ----------

    def destroy_node(self):
        try:
            GPIO.cleanup(self._pin)
        except Exception:
            pass
        return super().destroy_node()
