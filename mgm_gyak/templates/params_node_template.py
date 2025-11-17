#!/usr/bin/env python3
"""
Paraméterezett Node sablon.
- Megmutatja a declare/get paraméter használatát.
- Tipikusan launch-ból vagy YAML-ból állítod be.
"""
import rclpy
from rclpy.node import Node


class ParamNode(Node):
    def __init__(self):
        super().__init__('param_node')

        self.declare_parameter('path_topic_name', '/path')
        self.declare_parameter('max_size', 500)

        self.path_topic_name = self.get_parameter('path_topic_name').value
        self.max_size = int(self.get_parameter('max_size').value)

        self.get_logger().info(
            f'Params: path_topic_name={self.path_topic_name}, max_size={self.max_size}'
        )

        # ... ide jön a node funkciója (pub/sub/timer)


def main(args=None):
    rclpy.init(args=args)
    node = ParamNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
