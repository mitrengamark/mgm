"""PID_CONTROL node - PID szabályozó póz követéshez.

Magyarázat:
 - Feliratkozik a /goal_pose (célpozíció) és /odom (aktuális pozíció) topicokra.
 - PID szabályozó: Kp, Ki, Kd paraméterek (propor cionális, integrál, derivál).
 - Long itudinális (előre/hátra): távolság alapú sebességszámítás (max_speed korláttal).
 - Laterális (forgatás): szöghiba PID szabályozása (max_angular_speed korláttal).
 - Publikálja a /cmd_vel topicra (Twist) a sebességeket.
 - Timer: 10 Hz (0.1 sec) szabályozási ciklus.
"""

import rclpy
from rclpy.node import Node
import copy
import math

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

class PID_CONTROL(Node):
    def __init__(self):
        super().__init__('control_node')

        # PID paraméterek deklarálása és beolvasása.
        self.declare_parameter('Kp', 1.0)  # Propor cionális erősítés.
        self.declare_parameter('Ki', 0.0)  # Integrál erősítés.
        self.declare_parameter('Kd', 0.0)  # Derivál erősítés.
        self.Kp = self.get_parameter('Kp').value
        self.Ki = self.get_parameter('Ki').value
        self.Kd = self.get_parameter('Kd').value

        # Sebesség korlátok.
        self.declare_parameter('max_speed', 0.1)
        self.max_speed = self.get_parameter('max_speed').value
        self.declare_parameter('max_angular_speed', 0.3)
        self.max_angular_speed = self.get_parameter('max_angular_speed').value

        # Cél és aktuális pozíció tárolása.
        self.target_pose = PoseStamped()
        self.odom = Odometry()

        # PID állapotváltozók.
        self.integral = 0.0  # Integrál tag.
        self.previous_error = 0.0  # Előző hiba (deriválhoz).
        self.dt = 0.1  # Mintavételi idő (sec).

        # Subscriber: célpozíció fogadása.
        self.sub_target = self.create_subscription(PoseStamped, "/goal_pose", self.callback_target, 1)
        
        # Subscriber: aktuális odometria fogadása.
        self.sub_odom = self.create_subscription(Odometry, "/odom", self.callback_odom, 1)
        
        # Publisher: sebesség parancsok publikálása.
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 1)
        
        # Timer: 10 Hz szabályozási ciklus.
        self.timer = self.create_timer(self.dt, self.timer_callback)

    def callback_target(self, msg: PoseStamped):
        self.target_pose = msg

    def callback_odom(self, msg: Odometry):
        self.odom = msg

    def timer_callback(self):
        # Ha nincs cél vagy odometria, nem szabályozunk.
        if not self.target_pose or not self.odom:
            return

        # Távolság számítása (Euclidean norm).
        dx = self.target_pose.pose.position.x - self.odom.pose.pose.position.x
        dy = self.target_pose.pose.position.y - self.odom.pose.pose.position.y
        linear_error = math.sqrt(dx**2 + dy**2)

        # Célszög kiszámítása (irány a célhoz).
        target_yaw = math.atan2(dy, dx)
        
        # Cél orientáció (quaternion → yaw).
        g_goal = [
            self.target_pose.pose.orientation.x,
            self.target_pose.pose.orientation.y,
            self.target_pose.pose.orientation.z,
            self.target_pose.pose.orientation.w]
        goal_yaw = euler_from_quaternion(g_goal)[2]
        
        # Aktuális orientáció (quaternion → yaw).
        q = [self.odom.pose.pose.orientation.x,
             self.odom.pose.pose.orientation.y,
             self.odom.pose.pose.orientation.z,
             self.odom.pose.pose.orientation.w]
        current_yaw = euler_from_quaternion(q)[2]


        # Longitu dinális szabályozó (előre/hátra mozgás).
        linear_output = 0.0
        if linear_error > self.max_speed:
            linear_output = self.max_speed  # Sat uráció.
        elif linear_error > 0.1:
            linear_output = linear_error  # Propor cionális.

        # Laterális szabályozó (forgatás).
        # Ha mozgunk: irány a célhoz; ha állunk: végérték orientáció.
        if linear_output > 0.0:
            angular_error = target_yaw - current_yaw
        else:
            angular_error = goal_yaw - current_yaw

        # Szöghiba normalizálása [-π, π] tartományra.
        corrected_angle = math.atan2(math.sin(angular_error), math.cos(angular_error))

        # PID tagok frissítése.
        self.integral += corrected_angle * self.dt  # Integrál tag.
        derivative = (corrected_angle - self.previous_error) / self.dt  # Derivál tag.

        # PID kimenet kiszámítása.
        angular_output = self.Kp * corrected_angle + self.Ki * self.integral + self.Kd * derivative 

        # Szögsebesség korlátozása.
        angular_output = max(min(angular_output, self.max_angular_speed), -self.max_angular_speed)

        # Sebesség parancsok publikálása.
        cmd = Twist()
        cmd.linear.x = linear_output
        cmd.angular.z = angular_output
        self.pub_cmd.publish(cmd)

        # Előző hiba frissítése (derivál számításhoz).
        self.previous_error = linear_error

def main(args=None):
    # ROS inicializálás.
    rclpy.init(args=args)
    
    # Node létrehozása.
    pid_control_ = PID_CONTROL()
    
    # Eseményciklus indítása.
    rclpy.spin(pid_control_)
    
    # Erőforrások felszabadítása.
    pid_control_.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()