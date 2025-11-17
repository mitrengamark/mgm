import rclpy
from rclpy.node import Node
import copy
import math

from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
import tf2_ros
from tf_transformations import quaternion_from_euler

class ACK_ROBOT(Node):
    def __init__(self):
        super().__init__('ack_robot')

        self.declare_parameter("wheel_base", 0.3)
        self.wheel_base = self.get_parameter("wheel_base").value

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.cmd = Twist()
        self.sub_cmd = self.create_subscription(Twist, "/cmd_vel", self.callback_cmd, 1)

        self.pub_odom = self.create_publisher(Odometry, "/odom", 1)

        self.broadcaster = tf2_ros.TransformBroadcaster(self)

        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        front_wheel = TransformStamped()
        front_wheel.header.frame_id = "base_link"
        front_wheel.child_frame_id = "front_wheel"

        front_wheel.transform.translation.x = self.wheel_base
        front_wheel.transform.rotation.w = 1.0

        self.static_broadcaster.sendTransform(front_wheel)


        self.dt = 0.05 # 20Hz
        self.timer = self.create_timer(self.dt, self.timer_callback)


    def callback_cmd(self, msg:Twist):
        # self.get_logger().warn(f"Received cmd: linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f}")
        self.cmd = msg

    def timer_callback(self):
        
        self.x += self.cmd.linear.x * self.dt * math.cos(self.yaw)
        self.y += self.cmd.linear.x * self.dt * math.sin(self.yaw)
        self.yaw += self.cmd.angular.z / self.wheel_base * self.cmd.linear.x * self.dt

        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y

        q = quaternion_from_euler(0.0, 0.0, self.yaw)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        self.pub_odom.publish(odom)

        tf_stamped = TransformStamped()
        tf_stamped.header = odom.header
        tf_stamped.child_frame_id = odom.child_frame_id

        tf_stamped.transform.translation.x = odom.pose.pose.position.x
        tf_stamped.transform.translation.y = odom.pose.pose.position.y

        tf_stamped.transform.rotation = odom.pose.pose.orientation

        self.broadcaster.sendTransform(tf_stamped)

def main(args=None):
    rclpy.init(args=args)
    ack_robot_ = ACK_ROBOT()
    rclpy.spin(ack_robot_)
    ack_robot_.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()