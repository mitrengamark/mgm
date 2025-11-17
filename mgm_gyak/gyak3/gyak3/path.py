#!/usr/bin/env python3

"""PathCreator node - Odometriából path generálás.

Magyarázat:
 - Feliratkozik az odometria topicra és minden új pozícióból egy PoseStamped-et készít.
 - Ezeket egy Path üzenetbe fűzi, majd publikálja.
 - Ha a path túl hosszú lesz (max_size), régi elemeket töröl a lista elejéről (20%-ot).
 - Paraméterek: path_topic_name, odom_topic_name, max_size.
"""

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class PathCreator(Node):
    def __init__(self):
        super().__init__('odom_path')

        # Paraméterek deklarálása alapértelmezett értékekkel:
        # - path_topic_name: hova publikáljuk a path-t
        # - odom_topic_name: honnan olvassuk az odometriát
        # - max_size: hány pózból állhat max a path (ha 0, nincs limit)
        self.declare_parameter('path_topic_name', '/path')
        self.declare_parameter('odom_topic_name', '/odom')
        self.declare_parameter('max_size', 1500)

        # Paraméter értékek beolvasása.
        path_topic_name = self.get_parameter('path_topic_name').value
        odom_topic_name = self.get_parameter('odom_topic_name').value
        self.max_size = self.get_parameter('max_size').value

        # Path objektum inicializálása – ide gyűjtjük a pózokat.
        self.path = Path()

        # Publisher: path üzenet kiküldése.
        self.path_pub = self.create_publisher(Path, path_topic_name, 1)
        
        # Subscriber: odometria fogadása, minden üzenet érkezéskor hívódik az odom_cb.
        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic_name,
            self.odom_cb,
            1
        )

        self.get_logger().info(
            f"Publishing path on [{path_topic_name}] from odometry [{odom_topic_name}]"
        )

    def odom_cb(self, msg: Odometry):
        # Ha elértük a maximális méretet, töröljük a legrégebbi 20%-ot (trimmelés).
        if (len(self.path.poses) >= self.max_size) and (self.max_size > 0):
            del self.path.poses[0:int(self.max_size * 0.2)]

        # Új póz létrehozása az aktuális odometria alapján.
        pose = PoseStamped()
        pose.header = msg.header  # Időbélyeg és koordinátarendszer átvétele.
        pose.pose = msg.pose.pose  # Pozíció és orientáció átvétele.

        # Path header frissítése és új póz hozzáfűzése.
        self.path.header = msg.header
        self.path.poses.append(pose)

        # Frissített path publikálása.
        self.path_pub.publish(self.path)


def main(args=None):
    # ROS inicializálás.
    rclpy.init(args=args)
    
    # Node létrehozása.
    node = PathCreator()
    
    # Eseményciklus indítása.
    rclpy.spin(node)
    
    # Erőforrások felszabadítása.
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()