#!/usr/bin/env python3
"""
Minimal, kommentelt ROS2 Publisher sablon (rclpy).
Időzítővel adott frekvencián üzenetet küld.
"""
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')

        # Paraméter: küldési frekvencia (Hz)
        self.declare_parameter('rate_hz', 1.0)
        rate_hz = float(self.get_parameter('rate_hz').value)

        self.pub = self.create_publisher(String, '/chatter', 10)

        # Időzítő: 1/frekvencia másodperces periódus
        self.timer = self.create_timer(1.0 / max(rate_hz, 1e-3), self.on_timer)
        self.get_logger().info('Publisher ready on /chatter')

        self.counter = 0

    def on_timer(self):
        msg = String()
        msg.data = f'Hello #{self.counter}'
        self.pub.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = SimplePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
