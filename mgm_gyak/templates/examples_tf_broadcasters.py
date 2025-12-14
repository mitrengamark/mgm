# Példa: TF Broadcaster-ek használata (statikus és dinamikus)
# Cél: megmutatni, hogyan lehet TF kapcsolatokat publikálni:
#  - Statikus: map -> odom (nem változik)
#  - Dinamikus: odom -> base_link (időben változó)

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros

# MAGYAR KOMMENT: Statikus vs. dinamikus TF
# - Statikus TF: egyszer kerül publikálásra, nem változik (pl. 'map'→'odom').
# - Dinamikus TF: időben frissül, a robot mozgását írja le (pl. 'odom'→'base_link').

class TFBroadcastersPelda(Node):
    def __init__(self):
        super().__init__('tf_broadcasters_pelda')

        # Statikus TF (egyszeri publikálás): map -> odom
        self.static_brd = tf2_ros.StaticTransformBroadcaster(self)
        t_static = TransformStamped()
        t_static.header.frame_id = 'map'
        t_static.child_frame_id = 'odom'
        # Példa: azonos origó (nincs eltolás/forgatás)
        t_static.transform.rotation.w = 1.0
        self.static_brd.sendTransform(t_static)

        # Dinamikus TF: odom -> base_link
        self.brd = tf2_ros.TransformBroadcaster(self)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self.timer_cb)

    def timer_cb(self):
        # Egyszerű mozgás: előre haladás + kis fordulás
        self.x += 0.05
        self.yaw += 0.01

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        # yaw kvaternió: itt egyszerűsítve csak w=1.0 (nincs forgatás)
        t.transform.rotation.w = 1.0
        self.brd.sendTransform(t)

# Megjegyzés: ez a fájl sablon, nem szükséges futtatni a ZH-ban.
