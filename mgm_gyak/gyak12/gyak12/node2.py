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

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pub_points = self.create_publisher(PointCloud2, "/cloud", 10)
        self.pub_marker = self.create_publisher(Marker, "/marker", 10)
        self.pub_pose = self.create_publisher(PoseStamped, "/pose", 10)
        self.subscription = self.create_subscription(
            LaserScan, 
            "/scan", 
            self.scan_callback, 
            1
        )

        self.get_logger().info('TF Publisher node has been started')

    def scan_callback(self, msg: LaserScan):
        newPoint = []
        x = 0.0
        y = 0.0
        z = 0.0
        valid_points = 0
        
        for i in range(0, len(msg.ranges)):
            angle = msg.angle_min + i * msg.angle_increment
            
            if (msg.ranges[i] < msg.range_min or msg.ranges[i] > msg.range_max):
                continue
                
            x_temp = msg.ranges[i] * math.cos(angle)
            y_temp = msg.ranges[i] * math.sin(angle)
            z_temp = msg.ranges[i]
            newPoint.append([x_temp, y_temp, z_temp])

            x += x_temp
            y += y_temp
            z += z_temp
            valid_points += 1

        if valid_points == 0:
            return

        cloud = pcl2.create_cloud_xyz32(msg.header, newPoint)
        self.pub_points.publish(cloud)

        x_avg = x / valid_points
        y_avg = y / valid_points
        z_avg = z / valid_points

        marker = Marker()
        marker.header = msg.header
        marker.ns = "weight_point"
        marker.id = 0
        marker.type = marker.SPHERE
        marker.action = marker.ADD

        marker.pose.position.x = x_avg
        marker.pose.position.y = y_avg
        marker.pose.position.z = z_avg
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        marker.lifetime = Duration(seconds=0.1).to_msg()
        self.pub_marker.publish(marker)

        pose_local = PoseStamped()
        pose_local.header = msg.header
        pose_local.pose.position.x = x_avg
        pose_local.pose.position.y = y_avg
        pose_local.pose.position.z = z_avg
        pose_local.pose.orientation.w = 1.0
        if self.tf_buffer.can_transform("map", msg.header.frame_id, msg.header.stamp, Duration(seconds=0.1)):
            # Getting the transformation
            trans_scan2map = self.tf_buffer.lookup_transform(
                "map", 
                msg.header.frame_id, 
                msg.header.stamp
            )

            pose_map = tf2_geometry_msgs.do_transform_pose(pose_local, trans_scan2map)
            self.pub_pose.publish(pose_map)
        

def main(args=None):
    rclpy.init(args=args)
    zh_publisher_2 = ZhPub2()
    rclpy.spin(zh_publisher_2)
    zh_publisher_2.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()