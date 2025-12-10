import rclpy
from rclpy.executors import MultiThreadedExecutor

from .volume_controller_node import VolumeControllerNode
from .freq_controller_node import FrequencyControllerNode
from .ir_receiver_node import IRReceiverNode
from .display_node import DisplayNode

def main(args=None):
    print("Obsolete now, please use the ros2 launch command")
#    rclpy.init(args=args)

#    volume_controller_node = VolumeControllerNode()
#    frequency_controller_node = FrequencyControllerNode()
#    display_node = DisplayNode()
#    ir_receiver_node = IRReceiverNode()

#    executor = MultiThreadedExecutor()
#    executor.add_node(volume_controller_node)
#    executor.add_node(frequency_controller_node)
#    executor.add_node(display_node)
#    executor.add_node(ir_receiver_node)

#    print('Starting to spin the executor now')
#    try:
#        executor.spin()
#    except KeyboardInterrupt:
#        pass
#    finally:
#        volume_controller_node.destroy_node()
#        frequency_controller_node.destroy_node()
#        display_node.destroy_node()
#        ir_receiver_node.destroy_node()
#        rclpy.shutdown()

if __name__ == '__main__':
    main()