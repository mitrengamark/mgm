#!/usr/bin/env python3
"""
Feladat1 - proba1_node
======================
Funkcionalitás:
1) Feliratkozik /agent1/odom/ground_truth-ra (nav_msgs/Odometry)
   - Kiszámolja a 'yaw' szöget az orientáció quaternion-ból
2) Feliratkozik /path-ra (nav_msgs/Path)
   - Megkeresi a legközelebbi pontot az útvonalon
   - Publikálja /closest_point-ra (geometry_msgs/PoseStamped)
3) Kiszámolja a legközelebbi pont és saját pozíció közötti szöget
   - Eltávolítja a yaw-ot és publikálja (std_msgs/Float64)
"""
import rclpy
from rclpy.node import Node
import math

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
from tf_transformations import euler_from_quaternion


class Proba1Node(Node):
    def __init__(self):
        super().__init__('proba1_node')
        
        # ====================================================================
        # Állapotváltozók
        # ====================================================================
        # Saját pozíció és orientáció tárolása az Odometry-ből
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0  # radiánban, euler_from_quaternion-ból
        
        # Path tárolása (ha még nem érkezett, None)
        self.path = None
        
        # ====================================================================
        # Subscribers (feliratkozók)
        # ====================================================================
        # 1) Odometry feliratkozó: /agent1/odom/ground_truth
        self.sub_odom = self.create_subscription(
            Odometry,
            '/agent1/odom/ground_truth',
            self.callback_odom,
            10
        )
        
        # 2) Path feliratkozó: /path
        self.sub_path = self.create_subscription(
            Path,
            '/path',
            self.callback_path,
            10
        )
        
        # ====================================================================
        # Publishers (kiadók)
        # ====================================================================
        # 1) Legközelebbi pont: /closest_point (geometry_msgs/PoseStamped)
        self.pub_closest = self.create_publisher(
            PoseStamped,
            '/closest_point',
            10
        )
        
        # 2) Szögkülönbség: std_msgs/Float64
        #    (map x tengely és [saját_poz -> legközelebbi_pont] egyenes szöge - yaw)
        self.pub_angle = self.create_publisher(
            Float64,
            '/angle_diff',
            10
        )
        
        self.get_logger().info('proba1_node elindult. Várom az /agent1/odom/ground_truth és /path üzeneteket.')
    
    # ========================================================================
    # Callback: Odometry üzenet
    # ========================================================================
    def callback_odom(self, msg: Odometry):
        """
        Odometry üzenet feldolgozása:
        - Eltároljuk x, y pozíciót
        - Quaternion-ból kiszámoljuk a yaw szöget (euler_from_quaternion)
        """
        # Pozíció kinyerése
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Orientáció quaternion formában
        q = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]
        
        # Euler szögek kiszámítása: (roll, pitch, yaw)
        # Minket a yaw (z tengely körüli forgatás) érdekel
        (roll, pitch, yaw) = euler_from_quaternion(q)
        self.current_yaw = yaw
        
        # Ha van path, feldolgozzuk
        if self.path is not None:
            self.process_path()
    
    # ========================================================================
    # Callback: Path üzenet
    # ========================================================================
    def callback_path(self, msg: Path):
        """
        Path üzenet feldolgozása:
        - Eltároljuk az útvonalat
        - Ha van aktuális pozíció, feldolgozzuk
        """
        self.path = msg
        self.get_logger().info(f'Path érkezett: {len(self.path.poses)} pont')
        
        # Ha van aktuális pozíció, feldolgozzuk
        self.process_path()
    
    # ========================================================================
    # Path feldolgozás (legközelebbi pont keresése + szögszámítás)
    # ========================================================================
    def process_path(self):
        """
        Fő logika:
        1) Legközelebbi pont keresése az útvonalon
        2) Publikálás /closest_point-ra
        3) Szögszámítás és publikálás
        """
        # Ha nincs path, vagy üres, nem tudunk dolgozni
        if self.path is None or len(self.path.poses) == 0:
            return
        
        # ====================================================================
        # 1) LEGKÖZELEBBI PONT KERESÉSE
        # ====================================================================
        closest_distance = float('inf')  # végtelen kezdőérték
        closest_index = 0
        
        for i, pose_stamped in enumerate(self.path.poses):
            # Euklideszi távolság a path pont és a saját pozíció között
            dx = pose_stamped.pose.position.x - self.current_x
            dy = pose_stamped.pose.position.y - self.current_y
            distance = math.sqrt(dx**2 + dy**2)
            
            # Ha ez a legközelebbi eddig
            if distance < closest_distance:
                closest_distance = distance
                closest_index = i
        
        # Legközelebbi pont
        closest_pose = self.path.poses[closest_index]
        
        # ====================================================================
        # 2) LEGKÖZELEBBI PONT PUBLIKÁLÁSA
        # ====================================================================
        # Egyszerűen továbbítjuk a PoseStamped-et
        self.pub_closest.publish(closest_pose)
        
        # ====================================================================
        # 3) SZÖGSZÁMÍTÁS
        # ====================================================================
        # A feladat: 
        # - Legközelebbi pont és saját pozíció közötti egyenes szöge (map x tengelyhez képest)
        # - Ebből kivonjuk a saját yaw-ot
        
        # Delta x, y a legközelebbi pontig
        dx = closest_pose.pose.position.x - self.current_x
        dy = closest_pose.pose.position.y - self.current_y
        
        # atan2 megadja a szöget radiánban (-pi, pi tartományban)
        # atan2(dy, dx) = az [x tengely] és az [saját_poz -> célpont] vektor által bezárt szög
        angle_to_closest = math.atan2(dy, dx)
        
        # Szögkülönbség: angle_to_closest - current_yaw
        angle_diff = angle_to_closest - self.current_yaw
        
        # Normalizálás -pi..pi tartományba (opcionális, de hasznos)
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
        
        # Float64 üzenet publikálása
        angle_msg = Float64()
        angle_msg.data = angle_diff
        self.pub_angle.publish(angle_msg)
        
        # Debug log
        self.get_logger().info(
            f'Legközelebbi pont: ({closest_pose.pose.position.x:.2f}, '
            f'{closest_pose.pose.position.y:.2f}), '
            f'távolság: {closest_distance:.2f}m, '
            f'szögkülönbség: {math.degrees(angle_diff):.1f}°'
        )


def main(args=None):
    rclpy.init(args=args)
    node = Proba1Node()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
