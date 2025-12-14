# Példa: PointCloud2 transzformálása TF2-vel
# Cél: bemutatni a tf2_sensor_msgs.do_transform_cloud használatát.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pcl2
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from geometry_msgs.msg import TransformStamped

# MAGYAR KOMMENT: Felhő (PointCloud2) TF2 átalakítása
# - `create_cloud_xyz32(stamp, points)`: egyszerű XYZ32 formátumú felhőt készít.
# - A `header.frame_id` megadja, melyik frame-ben vannak a pontok (itt: 'base_scan').
# - A `can_transform()` és `lookup_transform()` segítségével lekérjük a TF-et.
# - `do_transform_cloud(cloud, tf)`: a felhőt átszámolja a cél frame-be.

class TF2CloudPelda(Node):
    def __init__(self):
        super().__init__('tf2_cloud_pelda')
        self.pub_cloud = self.create_publisher(PointCloud2, '/cloud', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.5, self.timer_cb)

    def timer_cb(self):
        # Lokális pontok (base_scan frame-ben)
        header = rclpy.time.Time()  # nem használjuk közvetlenül, csak szemléltetés
        points = [[0.5, 0.0, 0.0], [1.0, 0.1, 0.0]]
        cloud_local = pcl2.create_cloud_xyz32(self.get_clock().now().to_msg(), points)  # csak példa
        cloud_local.header.frame_id = 'base_scan'

        # Feltételezzük, hogy létezik map <- base_scan TF
        if self.tf_buffer.can_transform('map', 'base_scan', rclpy.time.Time(), rclpy.duration.Duration(seconds=0.2)):
            t = self.tf_buffer.lookup_transform('map', 'base_scan', rclpy.time.Time())
            cloud_map = do_transform_cloud(cloud_local, t)
            cloud_map.header.frame_id = 'map'
            self.pub_cloud.publish(cloud_map)
        else:
            self.get_logger().warn('TF nincs készen: map<-base_scan')

# Sablon: nem szükséges futtatni a ZH-ban.
