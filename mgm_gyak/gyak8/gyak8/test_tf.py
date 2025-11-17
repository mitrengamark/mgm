"""WheelPathHandler node - TF transzformációk felhasználása.

Magyarázat:
 - TF Buffer és Listener: transzformációk fogadása és lekérdezése.
 - Static TF Broadcaster: statikus transzformáció publikálása (base_link → safety_left).
 - can_transform: ellenőrzi, hogy létezik-e transzformáció két frame között.
 - lookup_transform: lekérdezi a transzformációt (TransformStamped üzenet).
 - Odometria publikálása: kerekek és safety_left pozíciója a cél frame-ben.
 - Timer: 10 Hz (0.1 sec) frekvenciával frissíti a transzformációkat.
"""

import rclpy
from rclpy.node import Node
import copy
import math

from nav_msgs.msg import Odometry
import tf2_ros


class WheelPathHandler(Node):
    def __init__(self):
        super().__init__('test_viz')

        # Paraméter: cél koordinátarendszer (pl. "odom").
        self.declare_parameter('target_frame', 'odom')
        self.target_frame = self.get_parameter('target_frame').value

        # TF Buffer: transzformációk tárolása.
        self.tfBuffer = tf2_ros.Buffer()
        
        # TF Listener: transzformációk fogadása a hálózatról.
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self)

        # Static TF Broadcaster: statikus transzformáció publikálása.
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        safety_transform = tf2_ros.TransformStamped()

        safety_transform.header.stamp = self.get_clock().now().to_msg()
        safety_transform.header.frame_id = "base_link"  # Szülő frame.
        safety_transform.child_frame_id = "safety_left"  # Gyerek frame.

        # Transzláció: 0.5 m balra (y irány).
        safety_transform.transform.translation.x = 0.0
        safety_transform.transform.translation.y = 0.5
        safety_transform.transform.translation.z = 0.0
        
        # Rotáció: nincs forgatás (egység quaternion).
        safety_transform.transform.rotation.x = 0.0
        safety_transform.transform.rotation.y = 0.0
        safety_transform.transform.rotation.z = 0.0
        safety_transform.transform.rotation.w = 1.0
        
        # Statikus transzformáció publikálása (egyszer).
        self.static_broadcaster.sendTransform(safety_transform)

        # Publisher: bal kerék odometria.
        self.pub_left = self.create_publisher(Odometry, '/odom_left', 1) 
        
        # Publisher: jobb kerék odometria.
        self.pub_right = self.create_publisher(Odometry, '/odom_right', 1) 

        # Publisher: safety_left odometria.
        self.pub_safety_left = self.create_publisher(Odometry, '/odom_safety_left', 1) 

        # Timer: 10 Hz frekvencia.
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        # Kerék pozíciók publikálása (TF-ből lekérdezve).
        self.publish_side_position("wheel_left_link", self.pub_left)
        self.publish_side_position("wheel_right_link", self.pub_right)
        self.publish_side_position("safety_left", self.pub_safety_left)

    def publish_side_position(self, wheel_frame: str, publisher):
        # Aktuális időpont (most).
        now = rclpy.time.Time()
        
        # Ellenőrzi, hogy létezik-e transzformáció a cél frame és a kerék frame között.
        # Timeout: 0.5 sec (várakozás a transzformációra).
        if self.tfBuffer.can_transform(self.target_frame, wheel_frame, time=now, timeout=rclpy.duration.Duration(seconds=0.5)):
            # Transzformáció lekérdezése (TransformStamped üzenet).
            trans = self.tfBuffer.lookup_transform(self.target_frame, wheel_frame, now)

            # Odometria üzenet létrehozása a transzformációból.
            odom = Odometry()
            odom.header.stamp = trans.header.stamp
            odom.header.frame_id = self.target_frame
            odom.child_frame_id = trans.child_frame_id
            odom.pose.pose.position.x = trans.transform.translation.x
            odom.pose.pose.position.y = trans.transform.translation.y
            odom.pose.pose.position.z = trans.transform.translation.z
            odom.pose.pose.orientation = trans.transform.rotation

            # Odometria publikálása.
            publisher.publish(odom)

def main(args=None):
    # ROS inicializálás.
    rclpy.init(args=args)
    
    # Node létrehozása.
    wheel_path_handler_ = WheelPathHandler()
    
    # Eseményciklus indítása.
    rclpy.spin(wheel_path_handler_)
    
    # Erőforrások felszabadítása.
    wheel_path_handler_.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()