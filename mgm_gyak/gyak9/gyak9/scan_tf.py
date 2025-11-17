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

        self.declare_parameter("map_frame", "odom")
        self.map_frame = self.get_parameter('map_frame').value

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)

        self.pointlist_dict = {}
        self.namespace_list = ['robot1', 'robot2']

        self.pub_cloud = self.create_publisher(PointCloud2, "/cloud", 1)
        self.sub_list = []
        for ns in self.namespace_list:
            sub = self.create_subscription(LaserScan, f"{ns}/scan", self.callback_scan, 1)
            self.sub_list.append(sub)

        self.timer = self.create_timer(1.0, self.timer_callback)
        

    def callback_scan(self, scan_in:LaserScan):

        newPoint = []
        for i in range(len(scan_in.ranges)):
            if (scan_in.range_min < scan_in.ranges[i] < scan_in.range_max):
                angle = scan_in.angle_min + scan_in.angle_increment * i
                x = scan_in.ranges[i] * math.cos(angle)
                y = scan_in.ranges[i] * math.sin(angle)
                z = 0.0
                newPoint.append([x, y, z])
        
        self.pointlist_dict[scan_in.header.frame_id] = newPoint

    def timer_callback(self):

        mapCloud = PointCloud2()
        for frame_id, points in self.pointlist_dict.items():
            self.get_logger().info(f"Frame_id: {frame_id}, cloud size: {len(points)}")

            cloud_header = Header()
            cloud_header.frame_id = frame_id
            cloud_header.stamp = self.get_clock().now().to_msg()
            localCloud = pcl2.create_cloud_xyz32(cloud_header, points)

            if self.tfBuffer.can_transform(self.map_frame, frame_id, rclpy.time.Time(), rclpy.duration.Duration(seconds=0.5)):
                trans_base2map = self.tfBuffer.lookup_transform(self.map_frame, frame_id, rclpy.time.Time())

                transformed_localCloud = do_transform_cloud(localCloud, trans_base2map)

                if mapCloud.width == 0:
                    mapCloud = transformed_localCloud
                else:
                    mapCloud = self.merge_pointclouds(mapCloud, transformed_localCloud)

        self.pub_cloud.publish(mapCloud)


    def merge_pointclouds(self, pc1_msg:PointCloud2, pc2_msg:PointCloud2):
        # Ensure both have valid fields
        if not pc2_msg.fields:
            raise ValueError("Second PointCloud2 message has no fields defined.")
        
        # Convert to lists of points
        points1 = list(pcl2.read_points(pc1_msg, skip_nans=True))
        points2 = list(pcl2.read_points(pc2_msg, skip_nans=True))

        # Merge points
        merged_points = points1 + points2

        self.get_logger().warn(f"length: {len(merged_points)}")
                

        # Recreate a new PointCloud2 message
        merged_msg = pcl2.create_cloud(
            pc2_msg.header,  # reuse header
            pc2_msg.fields,  # reuse fields
            merged_points
        )

        return merged_msg


def main(args=None):
    rclpy.init(args=args)
    scan_handler_ = ScanHandler()
    rclpy.spin(scan_handler_)
    scan_handler_.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()