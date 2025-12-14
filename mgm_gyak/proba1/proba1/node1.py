import rclpy
from rclpy.node import Node
import math
from nav_msgs.msg import Odometry, Path
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64  # gyak12/node1.py: 16. sor alapján

"""Készítsen egy node-t, mely feliratkozik egy „/agent1/odom/ground_truth” nevű „nav_msgs/Odometry” típusú topicra, melyből az orientáció alapján számolja ki a ‚yaw’ szöget.

Iratkozzon fel a „/path” nevű „nav_msgs/Path” típusú topicra. Az útvonal és a korábban megkapott pozíció alapján keresse meg a legközelebbi pontot az útvonalról, melyet egy „/closest_point” nevű „geometry_msgs/PoseStamped” típúsú topicra kiküld.

Számolja ki a legközelebbi pont és a saját pozíció által alkotott egyenes és a „map” frame x tengelye által bezárt szöget, ebből vonja ki a korábban kiszámolt „yaw” szöget és küldje ki „std_msgs/Float64” típusú üzenetként.

"""


class Node1(Node):
    def __init__(self):
        super().__init__('node1')

        # Változók inicializálása (gyak10/pure_pursuit.py: 40-41. sorok alapján)
        self.path = Path()
        self.current_position = None  # Aktuális pozíció tárolása
        self.current_yaw = 0.0  # Yaw szög tárolása

        self.sub_odom = self.create_subscription(Odometry, '/agent1/odom/ground_truth', self.odom_callback, 1)

        self.sub_path = self.create_subscription(Path, '/path', self.path_callback, 1)

        self.pub_pose = self.create_publisher(PoseStamped, '/closest_point', 1)
        # Publisher: szögkülönbség publikálása (gyak12/node1.py: 29. sor mintájára)
        self.pub_angle = self.create_publisher(Float64, '/angle_diff', 1)

    def path_callback(self, msg: Path):
        # Path tárolása (gyak10/pure_pursuit.py: 65. sor alapján)
        self.path = msg
        
        # Ellenőrzés: van-e pozíció és path (gyak10/pure_pursuit.py: 74-75. sorok alapján)
        if self.current_position is None or len(self.path.poses) < 1:
            return
        
        # Legközelebbi pont keresése a path-on (gyak10/pure_pursuit.py: 89-100. sorok alapján)
        index_closest_point = 0
        closest_distance = 9999.0
        for index in range(len(self.path.poses)):
            path_point = self.path.poses[index]
            dx_cal = path_point.pose.position.x - self.current_position.x
            dy_cal = path_point.pose.position.y - self.current_position.y
            distance_cal = math.sqrt(dx_cal**2 + dy_cal**2)
            if distance_cal < closest_distance:
                index_closest_point = index
                closest_distance = distance_cal
        
        # Legközelebbi pont publikálása
        closest_point = self.path.poses[index_closest_point]
        self.pub_pose.publish(closest_point)
        
        # Szögkülönbség számítása (gyak10/pure_pursuit.py: 124-136. sorok alapján)
        # A legközelebbi pont és saját pozíció közötti vektor szöge (map x tengely relativ)
        dx = closest_point.pose.position.x - self.current_position.x
        dy = closest_point.pose.position.y - self.current_position.y
        target_yaw = math.atan2(dy, dx)  # gyak10/pure_pursuit.py: 128. sor
        
        # Szöghiba: célirányszög - aktuális yaw (gyak10/pure_pursuit.py: 136. sor)
        angular_error = target_yaw - self.current_yaw
        
        # Float64 üzenet publikálása (gyak12/node1.py: 32. sor mintájára)
        angle_msg = Float64()
        angle_msg.data = angular_error
        self.pub_angle.publish(angle_msg)

    def odom_callback(self, msg: Odometry):
        # Pozíció tárolása
        self.current_position = msg.pose.pose.position
        
        # Quaternion -> Euler konverzió (gyak10/pure_pursuit.py: 130-134. sorok mintájára)
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        q = euler_from_quaternion([
            qx,
            qy,
            qz,
            qw
        ])
        self.current_yaw = q[2]  # Yaw szög tárolása


def main(args=None):
    # ROS inicializálás (gyak10/pure_pursuit.py: 193-203. sorok alapján)
    rclpy.init(args=args)
    
    # Node létrehozása
    node = Node1()
    
    # Eseményciklus indítása
    rclpy.spin(node)
    
    # Erőforrások felszabadítása
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

