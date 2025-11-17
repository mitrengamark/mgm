#!/usr/bin/env python3
"""
Feladat2 - proba2_node
======================
Feladat leírás:
- Feliratkozás: /agent1/odom/ground_truth (nav_msgs/Odometry)
- A kapott pozíció x és y koordinátáját felcseréljük és így jelenítünk meg egy 10x10x20 cm-es téglatestet (Marker).
  * A hosszú (20 cm) oldal mutasson a jármű haladási irányába -> scale.x = 0.20 (x tengely előre), scale.y = 0.10, scale.z = 0.10.
  * Orientációt átvesszük az Odometry quaternionból.
- 0.2 Hz-es (5 másodperc) timer: az utolsó Odometry üzenet időbélyegét (header.stamp) kiküldi egy std_msgs/Header üzenetben.

Megjegyzések:
- A Marker keretét az Odometry header.frame_id alapján állítjuk (pl. "map" vagy "odom").
- A Header üzenet frame_id mezőjébe az Odometry frame_id értékét tesszük.
"""
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from std_msgs.msg import Header

from tf_transformations import quaternion_from_euler  # Ha később szükséges lenne módosításhoz


class Proba2Node(Node):
    def __init__(self):
        super().__init__('proba2_node')

        # Állapot: utolsó Odometry (header és pose elkülönítve a timerhez)
        self.last_odom_header = None
        self.last_pose = None

        # Feliratkozás az Odometry-re
        self.sub_odom = self.create_subscription(
            Odometry,
            '/agent1/odom/ground_truth',
            self.callback_odom,
            10
        )

        # Marker publisher (felcserélt x,y pozícióval)
        self.pub_marker = self.create_publisher(Marker, '/swapped_box', 10)

        # Header publisher (időbélyeg periódikusan)
        self.pub_header = self.create_publisher(Header, '/last_odom_header', 10)

        # Timer 0.2 Hz (5 másodperc)
        self.timer_period = 5.0  # 1 / 0.2 Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info('proba2_node elindult, várom az Odometry üzeneteket...')

    # ------------------------------------------------------------------
    # Odometry callback: pozíció + orientáció feldolgozása, Marker publikálása
    # ------------------------------------------------------------------
    def callback_odom(self, msg: Odometry):
        self.last_odom_header = msg.header
        self.last_pose = msg.pose.pose

        # Eredeti pozíció
        ox = msg.pose.pose.position.x
        oy = msg.pose.pose.position.y
        oz = msg.pose.pose.position.z

        # Felcserélt koordináták a markerhez: x <-> y
        swapped_x = oy
        swapped_y = ox
        swapped_z = oz  # z változatlan

        marker = Marker()
        marker.header = msg.header  # Ugyanaz a frame + időbélyeg
        marker.ns = 'proba2'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose.position.x = swapped_x
        marker.pose.position.y = swapped_y
        marker.pose.position.z = swapped_z

        # Orientáció átvétele közvetlenül (a hosszú oldal x irányba áll így a headinget követi)
        marker.pose.orientation = msg.pose.pose.orientation

        # Méretek (méterben): 10x10x20 cm -> 0.10 x 0.10 x 0.20
        # Hosszú oldal előre (x tengely): legyen 0.20
        marker.scale.x = 0.20
        marker.scale.y = 0.10
        marker.scale.z = 0.10

        # Szín és átlátszóság
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.5
        marker.color.b = 1.0

        # Rövid életidő opcionálisan (0 = végtelen)
        # marker.lifetime = rclpy.duration.Duration(seconds=0.0).to_msg()

        self.pub_marker.publish(marker)

        self.get_logger().debug(
            f'Publikált Marker (swapped x,y): ({swapped_x:.2f}, {swapped_y:.2f})'
        )

    # ------------------------------------------------------------------
    # Timer callback: Header publikálása (utolsó Odometry időbélyeg)
    # ------------------------------------------------------------------
    def timer_callback(self):
        if self.last_odom_header is None:
            self.get_logger().warn('Még nem érkezett Odometry, nincs mit publikálni Header-ben.')
            return

        header_msg = Header()
        header_msg.stamp = self.last_odom_header.stamp
        header_msg.frame_id = self.last_odom_header.frame_id

        self.pub_header.publish(header_msg)
        self.get_logger().info(
            f'Header publikálva: stamp.sec={header_msg.stamp.sec}, frame_id={header_msg.frame_id}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = Proba2Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
