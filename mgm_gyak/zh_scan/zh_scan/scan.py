import rclpy
from rclpy.node import Node
import math
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from visualization_msgs.msg import MarkerArray, Marker 
from geometry_msgs.msg import Point # A pontok tárolásához a MarkerArray-ben

"""Készítsen egy node-t, mely feliratkozik egy „/agent1/scan” nevű „sensor_msgs/LaserScan” típusú
topicra, melyből vizsgálja meg, hogy a jármű előtt annak hossztengelyében, a
járműszélességével(~20cm) megegyező sávban, de maximum 5 méterre vannak-e visszaverődések.
Abban az esetben, ha igen, akkor küldjön ki egy bináris értéket „std_msgs/Bool” típusú üzenetként. (5
pont)

Az egyes visszaverődéseket kommunikálja ki 10 cm-es gömbként „visualization_msgs/MarkerArray”-
ként. (3 pont)
"""

class ScanNode(Node):
    def __init__(self):
        super().__init__('scan')

        self.sub_scan = self.create_subscription(LaserScan, '/agent1/scan', self.scan_callback, 1)

        self.pub_bin = self.create_publisher(Bool, '/bin', 1)

        self.bool_msg = Bool()

        self.max_distance = 5.0  # Maximális távolság a visszaverődésekhez (méter)
        self.max_lateral_deviation = 0.1  # Maximális oldalsó eltérés a jármű hossztengelyétől (méter)

        # Új Publisher: MarkerArray a LaserScan pontokhoz
        self.marker_pub = self.create_publisher(MarkerArray, '/scan_viz', 10)

        self.point = Point()
        self.marker_array = MarkerArray()
        self.marker = Marker()

    def scan_callback(self, msg: LaserScan):
        # A detektálás eredménye: True, ha találtunk akadályt az adott sávban
        obstacle_found = False

        # Az aktuális szög inicializálása
        angle = msg.angle_min # A kezdő szög

        points_for_viz = [] # Lista a MarkerArray számára

        # Számoljuk ki a kritikus szögablakot 5 méteres távolságra
        # theta_max = arcsin(0.1 / 5.0)
        # Bár ezt elég lenne egyszer kiszámolni, a callback-ben tartjuk a teljes körűségért.
        # FONTOS: rács ellenőrizni, hogy a LaserScan a base_link-re van-e rögzítve, ahol a 0 szög a hossztengely.
        self.critical_angle_rad = math.asin(self.max_lateral_deviation / self.max_distance)

        # Iterálás a LaserScan méréseken (ranges)
        for r in msg.ranges:
            
            # 1. Érvényességi ellenőrzés (gyak6 alapján): túl közel / túl távoli mérés kizárása
            # Szükséges továbbá ellenőrizni, hogy a távolság a 5m korláton belül van-e.
            if r > msg.range_min and r < msg.range_max and r <= self.max_distance:
                # 2. Szög ellenőrzése: A mérés a jármű hossztengelye menti sávban van-e?
                # Ellenőrizzük, hogy az abszolút szög a kritikus ablakon belül van-e
                if abs(angle) <= self.critical_angle_rad:
                    obstacle_found = True
                    break # Megtaláltuk az első akadályt, nincs szükség további ellenőrzésre
        
            # Szög frissítése a következő méréshez
            angle += msg.angle_increment

            # 1. Érvényességi ellenőrzés
            if r > msg.range_min and r < msg.range_max:
                
                # Poláris -> Descartes konverzió (mint a Node2-ben is láttuk) [3], [4]
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                z = 0.0 # Síkbeli LiDAR feltételezése
                
                # A pont hozzáadása a vizualizációs listához
                self.point.x = x
                self.point.y = y
                self.point.z = z
                points_for_viz.append(self.point)

        # 4. Bináris eredmény publikálása
        self.bool_msg.data = obstacle_found
        self.pub_bin.publish(self.bool_msg)

        if points_for_viz:
            
            # Fejléc és Frame ID átvétele az eredeti LaserScan üzenetből
            self.marker.header = msg.header 
            self.marker.ns = "scan_points_zh"
            self.marker.id = 0
            
            # Marker típusa: SPHERE_LIST (hatékony megjelenítés sok pontnál)
            self.marker.type = self.marker.SPHERE
            self.marker.action = self.marker.ADD
            
            # Méret beállítása: 10 cm-es gömb (0.1 m)
            self.marker.scale.x = 0.1 
            self.marker.scale.y = 0.1
            self.marker.scale.z = 0.1
            
            # Szín beállítása (pl. Zöld/Fehér)
            self.marker.color.a = 1.0 
            self.marker.color.r = 0.0
            self.marker.color.g = 1.0
            self.marker.color.b = 0.0
            
            # Pontok hozzáadása a markerhez
            self.marker.points = points_for_viz 
            
            self.marker_array.markers.append(self.marker)

        # Publikálás a /scan_viz topicra
        self.marker_pub.publish(self.marker_array)
            
            

def main(args=None):
    # ROS inicializálás.
    rclpy.init(args=args)
    
    # Node létrehozása.
    zh_scan = ScanNode()
    
    # Eseményciklus indítása.
    rclpy.spin(zh_scan)
    
    # Erőforrások felszabadítása.
    zh_scan.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()