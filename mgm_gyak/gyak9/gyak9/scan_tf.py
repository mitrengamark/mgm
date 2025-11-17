"""ScanHandler node - Több robot LaserScan aggregálás TF-fel.

Magyarázat:
 - Több robot (namespace: robot1, robot2) scan topicjaira feliratkozik.
 - Minden scan-t poláris → Descartes koordinátákká alakít.
 - PointCloud2 létrehozása minden robot scan-jéből (lokális frame-ben).
 - TF transzformáció: lokális frame → globális frame (pl. odom).
 - do_transform_cloud: PointCloud2 transzformálása (tf2_sensor_msgs).
 - Összes robot pontjának összefűzése (merge_pointclouds).
 - Globális PointCloud2 publikálása (1 Hz timer).
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter, ParameterType
import copy
import math

from geometry_msgs.msg import Point, PoseStamped
from sensor_msgs.msg import PointCloud2, LaserScan
from sensor_msgs_py import point_cloud2 as pcl2
from std_msgs.msg import Header
from visualization_msgs.msg import MarkerArray, Marker

import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

class ScanHandler(Node):
    def __init__(self):
        super().__init__('test_viz')

        # Paraméter: globális koordinátarendszer (pl. "odom").
        self.declare_parameter("map_frame", "odom")
        self.map_frame = self.get_parameter('map_frame').value

        # TF Buffer: transzformációk tárolása.
        self.tfBuffer = tf2_ros.Buffer()
        
        # TF Listener: transzformációk fogadása.
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)

        # Pontlisták tárolása (frame_id szerint).
        self.pointlist_dict = {}
        
        # Robot namespace-ek listája.
        self.namespace_list = ['robot1', 'robot2']

        # Publisher: globális PointCloud2 publikálása.
        self.pub_cloud = self.create_publisher(PointCloud2, "/cloud", 1)
        
        # Több robot scan topicjára feliratkozás (namespace-ekkel).
        self.sub_list = []
        for ns in self.namespace_list:
            # Például: "robot1/scan", "robot2/scan".
            sub = self.create_subscription(LaserScan, f"{ns}/scan", self.callback_scan, 1)
            self.sub_list.append(sub)

        # Timer: 1 Hz (1 sec) globális PointCloud publikálása.
        self.timer = self.create_timer(1.0, self.timer_callback)
        

    def callback_scan(self, scan_in:LaserScan):
        # Poláris → Descartes konverzió (ugyanaz, mint gyak6).
        newPoint = []
        for i in range(len(scan_in.ranges)):
            if (scan_in.range_min < scan_in.ranges[i] < scan_in.range_max):
                angle = scan_in.angle_min + scan_in.angle_increment * i
                x = scan_in.ranges[i] * math.cos(angle)
                y = scan_in.ranges[i] * math.sin(angle)
                z = 0.0
                newPoint.append([x, y, z])
        
        # Pontlista tárolása frame_id szerint (pl. "robot1/base_laser").
        self.pointlist_dict[scan_in.header.frame_id] = newPoint

    def timer_callback(self):
        # Globális PointCloud inicializálása.
        mapCloud = PointCloud2()
        
        # Végigmegyünk az összes robot pontlistáján.
        for frame_id, points in self.pointlist_dict.items():
            self.get_logger().info(f"Frame_id: {frame_id}, cloud size: {len(points)}")

            # Lokális PointCloud létrehozása (robot saját frame-jében).
            cloud_header = Header()
            cloud_header.frame_id = frame_id
            cloud_header.stamp = self.get_clock().now().to_msg()
            localCloud = pcl2.create_cloud_xyz32(cloud_header, points)

            # TF transzformáció lekérdezése (lokális → globális).
            if self.tfBuffer.can_transform(self.map_frame, frame_id, rclpy.time.Time(), rclpy.duration.Duration(seconds=0.5)):
                trans_base2map = self.tfBuffer.lookup_transform(self.map_frame, frame_id, rclpy.time.Time())

                # PointCloud2 transzformálása (do_transform_cloud: tf2_sensor_msgs).
                transformed_localCloud = do_transform_cloud(localCloud, trans_base2map)

                # Globális PointCloud összeépítése.
                if mapCloud.width == 0:
                    mapCloud = transformed_localCloud  # Első cloud.
                else:
                    mapCloud = self.merge_pointclouds(mapCloud, transformed_localCloud)

        # Globális PointCloud publikálása.
        self.pub_cloud.publish(mapCloud)


    def merge_pointclouds(self, pc1_msg:PointCloud2, pc2_msg:PointCloud2):
        # Ellenőrzi, hogy a második PointCloud valid-e.
        if not pc2_msg.fields:
            raise ValueError("Second PointCloud2 message has no fields defined.")
        
        # PointCloud2 → pontlisták konverziója.
        points1 = list(pcl2.read_points(pc1_msg, skip_nans=True))
        points2 = list(pcl2.read_points(pc2_msg, skip_nans=True))

        # Pontlisták összefűzése.
        merged_points = points1 + points2

        self.get_logger().warn(f"length: {len(merged_points)}")
                
        # Új PointCloud2 üzenet létrehozása az összefűzött pontokból.
        merged_msg = pcl2.create_cloud(
            pc2_msg.header,  # Header (időbélyeg, frame).
            pc2_msg.fields,  # Mező leírások (XYZ).
            merged_points
        )

        return merged_msg


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