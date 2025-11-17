#!/usr/bin/env python3
"""
Minimal, kommentelt ROS2 Subscriber sablon (rclpy).
ZH-n gyakran: egy adott topicra feliratkozás és az üzenet feldolgozása.
"""
import rclpy
from rclpy.node import Node

# Cseréld a típusra, amire feliratkozol (pl. std_msgs.msg.String)
from std_msgs.msg import String


class SimpleSubscriber(Node):
    def __init__(self):
        # A node neve, ami a grafban megjelenik
        super().__init__('simple_subscriber')

        # Feliratkozás létrehozása:
        #   - Üzenet típus (String)
        #   - Topic név ('/chatter')
        #   - Callback függvény (self.cb_msg)
        #   - QoS queue depth (10)
        self.sub = self.create_subscription(
            String,
            '/chatter',
            self.cb_msg,
            10
        )
        self.sub  # prevent unused variable warning

        self.get_logger().info('Subscriber ready on /chatter')

    def cb_msg(self, msg: String):
        # Itt dolgozod fel az üzenetet
        self.get_logger().info(f'Received: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = SimpleSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
