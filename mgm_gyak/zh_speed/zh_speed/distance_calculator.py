import rclpy
from rclpy.node import Node
import math
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Float64MultiArray
import tf2_ros
import tf2_geometry_msgs
from tf_transformations import quaternion_from_euler, euler_from_quaternion

"""Készítsen egy node-t, mely feliratkozik a „/agent1/odom/ground_truth” és a
„/agent2/odom/ground_truth” nevű „nav_msgs/Odometry” típusú topicokra és számolja ki a közöttük
lévő távolságot és kommunikálja ki egy „/tavolsag” nevű „std_msgs/Float64” típusú topic-ra 100 msként. (4 pont)

Számolja ki mindkét Odometry üzenet alapján a járművek orientációját/"yaw" értéket és egy
"std_msgs/Float64MultiArray" típusú üzenetbe összegyűjtve 100ms-ként küldje el. (4 pont)"""
class DistanceCalculator(Node):
    def __init__(self):
        # Node inicializálása
        super().__init__('distance_calculator')

        self.agent1 = self.create_subscription(Odometry, '/agent1/odom/ground_truth', self.agent1_callback, 1)
        self.agent2 = self.create_subscription(Odometry, '/agent2/odom/ground_truth', self.agent2_callback, 1)

        self.distance_publisher = self.create_publisher(Float64, '/tavolsag', 1)
        self.yaw_publisher = self.create_publisher(Float64MultiArray, '/yaw', 1)

        self.pos1 = None
        self.pos2 = None
        self.dist_msg = Float64()

        self.quat1 = None 
        self.quat2 = None
        self.yaw_array_msg = Float64MultiArray()

        self.timer = self.create_timer(0.1, self.timer_callback)

    def agent1_callback(self, msg: Odometry):
        """Fogadja és eltárolja az 1. robot pozícióját és orientációját."""
        self.pos1 = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        # Kvaternió eltárolása
        self.quat1 = msg.pose.pose.orientation

    def agent2_callback(self, msg: Odometry):
        """Fogadja és eltárolja a 2. robot pozícióját és orientációját."""
        self.pos2 = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        # Kvaternió eltárolása
        self.quat2 = msg.pose.pose.orientation

    def timer_callback(self):
        # 1. Ellenőrzés: Megkaptuk-e már mindkét robot pozícióját?
        if self.pos1 is None or self.pos2 is None:
            # Ha valamelyik hiányzik, várunk tovább
            return
        
        x1, y1 = self.pos1
        x2, y2 = self.pos2

        # 2. Távolság számítása Pitagorasz-tétellel (Euclidean távolság)
        dx = x2 - x1
        dy = y2 - y1
        distance = math.sqrt(dx**2 + dy**2)

        # 3. Publikálás
        self.dist_msg.data = distance
        self.distance_publisher.publish(self.dist_msg)

        if self.quat1 is None or self.quat2 is None:
            return
        
        q1 = (
            self.quat1.x,
            self.quat1.y,
            self.quat1.z,
            self.quat1.w
        )
        yaw1_radian = euler_from_quaternion(q1)[2]

        q2 = (
            self.quat2.x,
            self.quat2.y,
            self.quat2.z,
            self.quat2.w
        )
        yaw2_radian = euler_from_quaternion(q2)[2]

        self.yaw_array_msg.data = [yaw1_radian, yaw2_radian]
        self.yaw_publisher.publish(self.yaw_array_msg)

def main(args=None):
    # ROS inicializálás.
    rclpy.init(args=args)
    
    # Node létrehozása.
    zh_speed = DistanceCalculator()
    
    # Eseményciklus indítása.
    rclpy.spin(zh_speed)
    
    # Erőforrások felszabadítása.
    zh_speed.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()