import rclpy
from rclpy.node import Node
import copy
import math

from nav_msgs.msg import Odometry
import tf2_ros


class WheelPathHandler(Node):
    def __init__(self):
        super().__init__('test_viz')

        self.declare_parameter('target_frame', 'odom')
        self.target_frame = self.get_parameter('target_frame').value

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self)

        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        safety_transform = tf2_ros.TransformStamped()

        safety_transform.header.stamp = self.get_clock().now().to_msg()
        safety_transform.header.frame_id = "base_link"
        safety_transform.child_frame_id = "safety_left"

        safety_transform.transform.translation.x = 0.0
        safety_transform.transform.translation.y = 0.5
        safety_transform.transform.translation.z = 0.0
        safety_transform.transform.rotation.x = 0.0
        safety_transform.transform.rotation.y = 0.0
        safety_transform.transform.rotation.z = 0.0
        safety_transform.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform(safety_transform)

        self.pub_left = self.create_publisher(Odometry, '/odom_left', 1) 
        self.pub_right = self.create_publisher(Odometry, '/odom_right', 1) 

        self.pub_safety_left = self.create_publisher(Odometry, '/odom_safety_left', 1) 

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):

        self.publish_side_position("wheel_left_link", self.pub_left)
        self.publish_side_position("wheel_right_link", self.pub_right)
        self.publish_side_position("safety_left", self.pub_safety_left)

    def publish_side_position(self, wheel_frame: str, publisher):

        now = rclpy.time.Time()
        if self.tfBuffer.can_transform(self.target_frame, wheel_frame, time=now, timeout=rclpy.duration.Duration(seconds=0.5)):
            trans = self.tfBuffer.lookup_transform(self.target_frame, wheel_frame, now)

            odom = Odometry()
            odom.header.stamp = trans.header.stamp
            odom.header.frame_id = self.target_frame
            odom.child_frame_id = trans.child_frame_id
            odom.pose.pose.position.x = trans.transform.translation.x
            odom.pose.pose.position.y = trans.transform.translation.y
            odom.pose.pose.position.z = trans.transform.translation.z
            odom.pose.pose.orientation = trans.transform.rotation

            publisher.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    wheel_path_handler_ = WheelPathHandler()
    rclpy.spin(wheel_path_handler_)
    wheel_path_handler_.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()