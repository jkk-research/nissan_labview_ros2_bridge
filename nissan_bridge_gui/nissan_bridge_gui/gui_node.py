import rclpy
from rclpy.node import Node


class NissanBridgeGuiNode(Node):
    def __init__(self) -> None:
        super().__init__('nissan_bridge_gui')
        self.get_logger().info('nissan_bridge_gui skeleton node started')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NissanBridgeGuiNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
