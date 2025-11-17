"""DIFF_ROBOT node - Differenciális robot szimulátor.

Magyarázat:
 - Feliratkozik a /cmd_vel topicra (Twist: lineáris és szögsebesség).
 - Egyszerű differenciális robot kinematikai modell (x, y, yaw).
 - Publikálja az odometriát (/odom) és a TF transzformációt (odom → base_link).
 - Timer: 20 Hz (0.05 sec) frekvenciával frissíti a pozíciót.
 - Quaternion: yaw szögből számítva (tf_transformations.quaternion_from_euler).
"""

import rclpy
from rclpy.node import Node
import copy
import math

from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
import tf2_ros
from tf_transformations import quaternion_from_euler

class DIFF_ROBOT(Node):
    def __init__(self):
        super().__init__('diff_robot')

        # Robot pozíció állapotváltozók (kezdeti érték: origó).
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0  # Orientáció (rad).

        # Parancs tárolása (Twist: linear.x és angular.z).
        self.cmd = Twist()

        # TF Broadcaster: transzformációk publikálására (odom → base_link).
        self.broadcaster = tf2_ros.TransformBroadcaster(self)

        # Publisher: odometria publikálása.
        self.pub_odom = self.create_publisher(Odometry, "/odom", 1)
        
        # Subscriber: sebesség parancsok fogadása.
        self.sub_cmd = self.create_subscription(Twist, "/cmd_vel", self.callback_cmd, 1)

        # Timer: 20 Hz frekvencia (0.05 sec).
        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.timer_callback)


    def callback_cmd(self, msg:Twist):
        # Sebesség parancs fogadása (Twist: linear.x, angular.z).
        # self.get_logger().warn(f"Received cmd: linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f}")
        self.cmd = msg

    def timer_callback(self):
        # self.get_logger().info(f"Publishing odom: x={self.x:.2f}, y={self.y:.2f}, yaw={self.yaw:.2f}")
        
        # Differenciális robot kinematika:
        # x += v * dt * cos(yaw)
        # y += v * dt * sin(yaw)
        # yaw += omega * dt
        self.x += self.cmd.linear.x * self.dt * math.cos(self.yaw)
        self.y += self.cmd.linear.x * self.dt * math.sin(self.yaw)
        self.yaw += self.cmd.angular.z * self.dt

        # Odometry üzenet létrehozása.
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()  # Aktuális időbélyeg.
        odom.header.frame_id = "odom"  # Globális koordinátarendszer.
        odom.child_frame_id = "base_link"  # Robot koordinátarendszer.
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y

        # Orientáció (yaw) → quaternion konverzió.
        q = quaternion_from_euler(0, 0, self.yaw)  # (roll, pitch, yaw).
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        
        # Odometry publikálása.
        self.pub_odom.publish(odom)

        # TF transzformáció létrehozása (odom → base_link).
        tf_stamped = TransformStamped()
        tf_stamped.header = odom.header
        tf_stamped.child_frame_id = odom.child_frame_id

        # Transzláció (pozíció) átmásolása.
        tf_stamped.transform.translation.x = odom.pose.pose.position.x
        tf_stamped.transform.translation.y = odom.pose.pose.position.y
        tf_stamped.transform.translation.z = odom.pose.pose.position.z

        # Rotáció (quaternion) átmásolása.
        tf_stamped.transform.rotation = odom.pose.pose.orientation

        # TF publikálása.
        self.broadcaster.sendTransform(tf_stamped)                                                                                                                                        

def main(args=None):
    # ROS inicializálás.
    rclpy.init(args=args)
    
    # Node létrehozása.
    diff_robot_ = DIFF_ROBOT()
    
    # Eseményciklus indítása.
    rclpy.spin(diff_robot_)
    
    # Erőforrások felszabadítása.
    diff_robot_.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()