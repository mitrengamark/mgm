"""ScanHandler node - LaserScan feldolgozás és vizualizáció.

Magyarázat:
 - Feliratkozik a LaserScan topicra (többnyiás mérés, LiDAR/LIDAR adatok).
 - Poláris koordinátákból (távolság, szög) Descartes-koordinátákká (x,y,z) alakít.
 - PointCloud2: 3D ponthalmaz publikálása (RViz-ben megjeleníthető).
 - MarkerArray: SPHERE_LIST marker - összes pont vizualizációja.
 - Legközelebbi pont: a legkisebb távolságú mérési pont kiemelése PoseStamped üzenetben.
 - Szűrés: csak a valid tartományban lévő mérések kerülnek feldolgozásra.
"""

import rclpy
from rclpy.node import Node
import copy
import math

from geometry_msgs.msg import Point, PoseStamped
from sensor_msgs.msg import PointCloud2, LaserScan
from sensor_msgs_py import point_cloud2 as pcl2
from visualization_msgs.msg import MarkerArray, Marker

class ScanHandler(Node):
    def __init__(self):
        super().__init__('test_viz')

        # Publisher: PointCloud2 publikálása (3D ponthalmaz).
        self.pub_cloud = self.create_publisher(PointCloud2, "/cloud", 1)

        # Publisher: MarkerArray vizualizációhoz (SPHERE_LIST).
        self.pub_marker = self.create_publisher(MarkerArray, "/viz", 1)

        # Publisher: legközelebbi pont publikálása.
        self.closest_point_pub = self.create_publisher(PoseStamped, "/closest_point", 1)
        
        # Subscriber: LaserScan fogadása.
        self.sub = self.create_subscription(LaserScan, "/scan", self.callback_scan, 1)

    def callback_scan(self, scan_in:LaserScan):
        # Új pontlista létrehozása (Descartes-koordináták).
        newPoint = []
        
        # Végigmegyünk az összes LaserScan mérésen.
        for i in range(len(scan_in.ranges)):
            
            # Szűrés: csak a valid tartományban lévő mérések (range_min és range_max között).
            if ((scan_in.range_min < scan_in.ranges[i]) and (scan_in.ranges[i] < scan_in.range_max)):
                # Szög kiszámítása: angle_min + i * angle_increment.
                angle = scan_in.angle_min + scan_in.angle_increment * i
                
                # Poláris → Descartes koordináta-transzformáció:
                # x = r * cos(θ), y = r * sin(θ), z = 0 (síkbeli scan).
                x = scan_in.ranges[i] * math.cos(angle)
                y = scan_in.ranges[i] * math.sin(angle)
                z = 0.0
                newPoint.append([x, y, z])
        
        # MarkerArray létrehozása a pontok vizualizációjához.
        marker_array_ = MarkerArray()
        
        marker = Marker()
        marker.header = scan_in.header  # Időbélyeg és frame (pl. "laser").
        marker.ns = "scan"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST  # Több gömb egyetlen markerben (hatékony).
        marker.action = Marker.ADD

        # Méret: minden gömb 0.05 m átmérőjű.
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05

        # Szín: kék (RGBA).
        marker.color.a = 1.0
        marker.color.b = 1.0

        # Pontok hozzáadása a markerhez.
        for point in newPoint:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            p.z = point[2]

            marker.points.append(p)

        # Lifetime: 0.15 sec után eltűnik, ha nincs frissítés.
        marker.lifetime = rclpy.duration.Duration(seconds=0.15).to_msg()

        marker_array_.markers.append(marker)

        # MarkerArray publikálása.
        self.pub_marker.publish(marker_array_)

        # PointCloud2 létrehozása: 3D ponthalmaz (XYZ formátum, float32).
        # create_cloud_xyz32: segédfüggvény a sensor_msgs_py csomagból.
        localCloud = pcl2.create_cloud_xyz32(scan_in.header, newPoint)
        self.pub_cloud.publish(localCloud)

        # Legközelebbi pont keresése: a legkisebb távolságú mérés.
        closest_dist = 1000  # Nagy kezdőérték (minden mérés kisebb ennél).
        closest_dist_angle = 0

        for i in range(len(scan_in.ranges)):
            dist = scan_in.ranges[i] 
            if dist < closest_dist:
                closest_dist = dist
                closest_dist_angle = scan_in.angle_min + i * scan_in.angle_increment

        # PoseStamped üzenet létrehozása a legközelebbi pont számára.
        closest_point = PoseStamped()
        closest_point.header = scan_in.header
        
        # Poláris → Descartes koordináták.
        closest_point.pose.position.x = closest_dist * math.cos(closest_dist_angle)
        closest_point.pose.position.y = closest_dist * math.sin(closest_dist_angle)
        closest_point.pose.position.z = 0.0
        closest_point.pose.orientation.w = 1.0  # Nincs forgatás (egység quaternion).
        
        # Legközelebbi pont publikálása.
        self.closest_point_pub.publish(closest_point)




def main(args=None):
    # ROS inicializálás.
    rclpy.init(args=args)
    
    # Node létrehozása.
    scan_handler_ = ScanHandler()
    
    # Eseményciklus indítása.
    rclpy.spin(scan_handler_)
    
    # Erőforrások felszabadítása.
    scan_handler_.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()