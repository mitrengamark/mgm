import rclpy
import random
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Path # Új import
from geometry_msgs.msg import PoseStamped, Quaternion, Point
from std_msgs.msg import Header
import math
from tf_transformations import quaternion_from_euler # Kvaternió konverzió

"""Készítsen egy node-t, mely a feliratkozik egy “agent1/objects” nevű topicra. Megérkező
objektumok közül egy 0.1 Hz-es időzítő segítségével válaszunk ki véletlenszerűen két pontot.
(3 pont)

Számolja ki a két pont alkotta szakasz harmadoló pontjait, amiket „nav_msgs/Path” típusú
üzenetként továbbítson egy topic-on. Az irányvektorok a harmadoló pontok esetében legyen
merőleges a szakaszra."""


class RandomObjectSelectorNode(Node):
    def __init__(self):
        super().__init__('node2')

        # Tároló a legutóbb beérkezett objektumlistának
        self.last_marker_array = [] 

        self.sub_obj = self.create_subscription(MarkerArray, 'agent1/objects', self.obj_callback, 1)

        self.timer = self.create_timer(10.0, self.timer_callback)

        # Publikáló a Path üzenethez
        self.path_publisher = self.create_publisher(
            Path,
            'agent1/trisection_path',
            1
        )
        # Az A és B pontok (Marker objektumok) tárolására szolgáló attribútumok
        self.point_A = None
        self.point_B = None

    def obj_callback(self, msg: MarkerArray):
        # Frissítjük a legutóbb beérkezett objektumlistát
        self.last_marker_array = msg.markers

    def timer_callback(self):
        if len(self.last_marker_array) < 2:
            return  # Nincs elég objektum a kiválasztáshoz
        
        # Véletlenszerűen 2 egyedi Marker objektum kiválasztása a listából
        selected_markers = random.sample(self.last_marker_array, 2)
            
        # Az eredmény tárolása a későbbi feladatokhoz:
        self.point_A = selected_markers[0]
        self.point_B = selected_markers[1]

        # Objektumok pozícióinak kinyerése
        pos_A = self.point_A.pose.position
        pos_B = self.point_B.pose.position

        # 2. Vektor V kiszámítása (P_B - P_A)
        dx = pos_B.x - pos_A.x
        dy = pos_B.y - pos_A.y
        dz = pos_B.z - pos_A.z # Feltételezve, hogy a Z tengelyt is figyelembe vesszük

        # 3. A szakasz (AB) irányának (yaw) meghatározása
        # atan2 a megfelelő függvény a síknegyedek kezelésére
        segment_yaw = math.atan2(dy, dx)

        # 4. Merőleges irány meghatározása
        # A merőleges vektor a szegmens irányához képest +90 fokkal (+pi/2 radiánnal) elforgatott
        perpendicular_yaw = segment_yaw + math.pi / 2.0

        # 5. Harmadoló pontok (T1 és T2) számítása
        
        # T1 (1/3 arányban)
        x_T1 = pos_A.x + dx / 3.0
        y_T1 = pos_A.y + dy / 3.0
        z_T1 = pos_A.z + (pos_B.z - pos_A.z) / 3.0
        
        # T2 (2/3 arányban)
        x_T2 = pos_A.x + 2.0 * dx / 3.0
        y_T2 = pos_A.y + 2.0 * dy / 3.0
        z_T2 = pos_A.z + 2.0 * (pos_B.z - pos_A.z) / 3.0

        # 6. Orientáció (Kvaternió) generálása a merőleges szögből
        # Roll és Pitch 0, csak Yaw forog
        quaternion_list = quaternion_from_euler(0.0, 0.0, perpendicular_yaw)
        q = Quaternion(x=quaternion_list[0], y=quaternion_list[1], z=quaternion_list[2], w=quaternion_list[3])

        # 7. nav_msgs/Path üzenet létrehozása és feltöltése
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        # Feltételezve, hogy a markerek a "map" frame-ben vannak
        path_msg.header.frame_id = "map" 

        # PoseStamped üzenetek létrehozása T1 és T2 pontokhoz
        
        # T1 PoseStamped
        pose_stamped_T1 = PoseStamped()
        pose_stamped_T1.header = path_msg.header
        pose_stamped_T1.pose.position = Point(x=x_T1, y=y_T1, z=z_T1)
        pose_stamped_T1.pose.orientation = q
        
        # T2 PoseStamped
        pose_stamped_T2 = PoseStamped()
        pose_stamped_T2.header = path_msg.header
        pose_stamped_T2.pose.position = Point(x=x_T2, y=y_T2, z=z_T2)
        pose_stamped_T2.pose.orientation = q
        
        # Hozzáadás a Path listához
        path_msg.poses.append(pose_stamped_T1)
        path_msg.poses.append(pose_stamped_T2)
        
        # 8. Publikálás
        self.path_publisher.publish(path_msg)

def main(args=None):
    # ROS inicializálás.
    rclpy.init(args=args)
    
    # Node létrehozása.
    zh = RandomObjectSelectorNode()
    
    # Eseményciklus indítása.
    rclpy.spin(zh)
    
    # Erőforrások felszabadítása.
    zh.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()