import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2 as pc2

# MAGYAR KOMMENT: PointCloud2 létrehozása és publikálása
# - A PointCloud2 egy táblázatos formátumú pontfelhő (mezők: x,y,z stb.).
# - A `PointField` mezők típusa és elrendezése határozza meg a bináris adat struktúráját.
# - A `pc2.create_cloud()` segít egy Python listából (pontok) komplett PointCloud2-t csinálni.

class PointCloud2Example(Node):
    """Példa: egyszerű PointCloud2 publikálása három ponttal (MAGYARUL).

    Lépések:
    1) Definiáljuk a mezőket (x,y,z float32).
    2) Összeállítjuk a fejlécet (frame_id: 'map').
    3) Létrehozzuk a felhőt és elküldjük a '/points' topikra.
    """
    def __init__(self):
        super().__init__('examples_pointcloud2')
        self.pub = self.create_publisher(PointCloud2, '/points', 10)
        self.timer = self.create_timer(1.0, self.timer_cb)

    def timer_cb(self):
        # Fejléc: megadjuk, hogy melyik koordináta-rendszerben vannak a pontok
        header = Header()
        header.frame_id = 'map'

        # Mezők: x,y,z float32; az offsetek 4 byte-onként nőnek
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        # Pontok listája (x,y,z): egyszerű háromszög a síkban
        points = [
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0),
            (1.0, 1.0, 0.0),
        ]

        # Felhő létrehozása és publikálása
        cloud = pc2.create_cloud(header, fields, points)
        self.pub.publish(cloud)
        self.get_logger().info('3 pontos felhő publikálva a map frame-ben')
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
import sensor_msgs_py.point_cloud2 as pcl2
import math

class PointCloudFromScan(Node):
    """Example: Convert LaserScan to PointCloud2 (XYZ32)."""
    def __init__(self):
        super().__init__('examples_pointcloud2')
        self.pub_cloud = self.create_publisher(PointCloud2, '/cloud', 10)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.cb, 10)

    def cb(self, scan: LaserScan):
        pts = []
        for i, r in enumerate(scan.ranges):
            if r <= scan.range_min or r >= scan.range_max:
                continue
            angle = scan.angle_min + i * scan.angle_increment
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            z = 0.0
            pts.append([x, y, z])
        cloud = pcl2.create_cloud_xyz32(scan.header, pts)
        self.pub_cloud.publish(cloud)


def main():
    rclpy.init()
    n = PointCloudFromScan()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()
