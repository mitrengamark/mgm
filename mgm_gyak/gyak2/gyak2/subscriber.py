import rclpy
from rclpy.node import Node
from std_msgs.msg import String

"""Egyszerű subscriber példa – JELENLEG HIÁNYOS.

Magyarázat:
 - create_subscription(msg_type, topic, callback, qos_depth) formában kell.
 - Itt hiányoznak a paraméterek, így nem fog üzenetet kapni.
 - Példa teljes sorra:
   self.subscription = self.create_subscription(String, 'chatter', self.listener_callback, 10)
 - A callback-ben (listener_callback) lehet a kapott üzenetet kiírni.
"""

class Subscriber_Node(Node):
    def __init__(self):
        super().__init__('subscriber_node')

        # HIÁNYOS: a subscription paraméterek nélkül van meghívva.
        # Teljesen helyes forma kommentben:
        # self.subscription = self.create_subscription(String, 'chatter', self.listener_callback, 10)
        self.subscription = self.create_subscription()



def main(args=None):
    rclpy.init(args=args)

    subscriber = Subscriber_Node()

    rclpy.spin(subscriber)

    rclpy.shutdown()

if __name__ == '__main__':
    main()