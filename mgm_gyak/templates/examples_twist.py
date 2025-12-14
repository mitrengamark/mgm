# Példa: geometry_msgs/Twist publikálása (cmd_vel)
# Cél: bemutatni a lineáris és szögsebesség parancs kiküldését.

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TwistPelda(Node):
    def __init__(self):
        super().__init__('twist_pelda')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.declare_parameter('linear_x', 0.1)
        self.declare_parameter('angular_z', 0.2)
        self.timer = self.create_timer(0.5, self.timer_cb)

    def timer_cb(self):
        cmd = Twist()
        cmd.linear.x = float(self.get_parameter('linear_x').value)
        cmd.angular.z = float(self.get_parameter('angular_z').value)
        self.pub.publish(cmd)
        self.get_logger().info(f'cmd_vel: v={cmd.linear.x:.2f} az={cmd.angular.z:.2f}')

# Sablon: nem kell futtatni a ZH-ban.
 # MAGYAR KOMMENT: Twist (cmd_vel) publikálás
 # - A `linear.x` az előre/hátra sebesség (m/s).
 # - Az `angular.z` a körbefordulás szögsebessége (rad/s).
 # - Paraméterekkel állítható: `linear_x`, `angular_z`.
