"""ZhPub2 node - LaserScan feldolgozás, súlyponti pont számítás és TF transzformáció.

Magyarázat:
 - Feliratkozik a /scan topicra, poláris → Descartes koordináta-transzformáció.
 - PointCloud2 publikálása (összes valid pont).
 - Súlyponti pont (centroid): átlag x, y, z koordináták.
 - Marker vizualizáció: súlyponti pont (zöld SPHERE).
 - TF transzformáció: lokális frame → "map" frame.
 - PoseStamped publikálása: súlyponti pont globális koordinátákban.
"""

import math
import copy
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from sensor_msgs.msg import PointCloud2, LaserScan
import sensor_msgs_py.point_cloud2 as pcl2
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped

import tf2_ros
import tf2_geometry_msgs


class ZhPub2(Node):
    def __init__(self):
        super().__init__('zh_node2')

        # TF Buffer és Listener: transzformációk lekérdezéséhez.
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publisher: PointCloud2, Marker, PoseStamped.
        self.pub_points = self.create_publisher(PointCloud2, "/cloud", 10)
        self.pub_marker = self.create_publisher(Marker, "/marker", 10)
        self.pub_pose = self.create_publisher(PoseStamped, "/pose", 10)
        
        # Subscriber: LaserScan fogadása.
        self.subscription = self.create_subscription(
            LaserScan, 
            "/scan", 
            self.scan_callback, 
            1
        )

        self.get_logger().info('TF Publisher node has been started')

    def scan_callback(self, msg: LaserScan):
        # Pontlista és súlyponti koordináták inicializálása.
        newPoint = []
        x = 0.0
        y = 0.0
        z = 0.0
        valid_points = 0
        
        # Végigmegyünk a LaserScan méréseken.
        for i in range(0, len(msg.ranges)):
            angle = msg.angle_min + i * msg.angle_increment
            
            # Szűrés: csak valid tartományban lévő mérések.
            if (msg.ranges[i] < msg.range_min or msg.ranges[i] > msg.range_max):
                continue
            
            # Poláris → Descartes koordináták.
            x_temp = msg.ranges[i] * math.cos(angle)
            y_temp = msg.ranges[i] * math.sin(angle)
            z_temp = msg.ranges[i]  # Megjegyzés: általában z=0, itt z=távolság.
            newPoint.append([x_temp, y_temp, z_temp])

            # Súlyponti koordináták összegzése.
            x += x_temp
            y += y_temp
            z += z_temp
            valid_points += 1

        # Ha nincs valid pont, kilépünk.
        if valid_points == 0:
            return

        # PointCloud2 létrehozása és publikálása.
        cloud = pcl2.create_cloud_xyz32(msg.header, newPoint)
        self.pub_points.publish(cloud)

        # Súlyponti pont számítása (átlag).
        x_avg = x / valid_points
        y_avg = y / valid_points
        z_avg = z / valid_points

        # Marker létrehozása: súlyponti pont vizualizációja.
        marker = Marker()
        marker.header = msg.header
        marker.ns = "weight_point"
        marker.id = 0
        marker.type = marker.SPHERE
        marker.action = marker.ADD

        # Pozíció: súlyponti koordináták.
        marker.pose.position.x = x_avg
        marker.pose.position.y = y_avg
        marker.pose.position.z = z_avg
        marker.pose.orientation.w = 1.0

        # Méret: 0.1 m átmérőjű gömb.
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        # Szín: zöld (RGBA).
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        # Lifetime: 0.1 sec.
        marker.lifetime = Duration(seconds=0.1).to_msg()
        self.pub_marker.publish(marker)

        # Lokális PoseStamped létrehozása (scan frame-ben).
        pose_local = PoseStamped()
        pose_local.header = msg.header
        pose_local.pose.position.x = x_avg
        pose_local.pose.position.y = y_avg
        pose_local.pose.position.z = z_avg
        pose_local.pose.orientation.w = 1.0
        
        # TF transzformáció: lokális → "map" frame.
        if self.tf_buffer.can_transform("map", msg.header.frame_id, msg.header.stamp, Duration(seconds=0.1)):
            # Transzformáció lekérdezése.
            trans_scan2map = self.tf_buffer.lookup_transform(
                "map", 
                msg.header.frame_id, 
                msg.header.stamp
            )

            # Póz transzformálása globális frame-be.
            pose_map = tf2_geometry_msgs.do_transform_pose(pose_local, trans_scan2map)
            
            # Globális póz publikálása.
            self.pub_pose.publish(pose_map)
        

def main(args=None):
    # ROS inicializálás.
    rclpy.init(args=args)
    
    # Node létrehozása.
    zh_publisher_2 = ZhPub2()
    
    # Eseményciklus indítása.
    rclpy.spin(zh_publisher_2)
    
    # Erőforrások felszabadítása.
    zh_publisher_2.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()