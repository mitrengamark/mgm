import rclpy
from rclpy.node import Node
import math
import random
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.duration import Duration
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs
from std_msgs.msg import Int32


"""Készítsen egy node-t, mely létrehoz egy objektum listát, amely az alábbi képletek alapján
legenerál 1000 db objektumot:
- i = [1,100] ∈ N
- x = 10 * cos( i ) + 0.1 * i - 10
- y = 10 / i - 0.5
- z = sqrt(x*x + y*y)
A kapott objektum lista az “map” frame-ban van értelmezze.
Egy időzítő segítségével az objektum listát küldje ki “agent1/objects” nevű topicra,
“visualization_msgs/MarkerArray” típusú üzenetben. Az időzítő 1000ms -os időzítéssel
dolgozzon. Az objektumok 15 centiméteres átmérőjű gömbök legyenek. A színük legyen
véletlenszerű.
(5 pont)

Íratkozzon fel az “/agent1/odom/ground_truth” topicra és a járműpozíciója alapján keresse meg
a legközelebbi objektumot. A legközelebbi objektum pozícióját az „agent1/base_link”
koordináta-rendszerbe konvertálja át és küldje ki azt, egy „geometry_msgs/PoseStamped”
típusú üzenetként.
(4 pont)

Számolja ki azoknak az objektumoknak a számát, ami a második sík negyedben(++) találhatók
és számukat egy „std_msgs/Int32” típusú üzenetként küldje ki.
(2 pont)
"""


class ObjectPublisherNode(Node):
    def __init__(self):
        super().__init__('node1')

        self.pub_obj = self.create_publisher(MarkerArray, 'agent1/objects', 1)

        self.timer = self.create_timer(1.0, self.timer_callback)

        self.sub_odom = self.create_subscription(Odometry, '/agent1/odom/ground_truth', self.odom_callback, 1)

        self.pub_odom = self.create_publisher(PoseStamped, 'agent1/closest_object', 1)

        # TF Buffer és Listener inicializálása
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub_object_count = self.create_publisher(Int32, 'object_count_second_quadrant', 1)

        # Előre generált objektumok tárolója (odom callback előtt is létezzen).
        self.markers = []

    def odom_callback(self, msg: Odometry):
        # Ha még nincs objektumlista, nincs mit keresni.
        if not self.markers:
            return

        # 1. Robot pozíció kinyerése
        robot_pose = msg.pose.pose.position

        min_distance = float('inf')
        closest_marker_pose = None

        # 2. Legközelebbi objektum keresése a 1000 marker közül
        # Feltételezzük, hogy self.markers listában tároljuk az előre generált Marker üzeneteket.
        for marker in self.markers:
            obj_pos = marker.pose.position
            
            # Euklideszi távolság számítása (Pitagorasz-tétel)
            distance = math.hypot(robot_pose.x - obj_pos.x, robot_pose.y - obj_pos.y)
            
            if distance < min_distance:
                min_distance = distance
                closest_marker_pose = marker.pose
                # Feltételezzük, hogy az objektumok a map frame-ben vannak (az előző lépés szerint)
        
        if closest_marker_pose is None:
            self.get_logger().warn('Nincs objektum a listában.')
            return

        # 3. Transzformáció előkészítése
        # Az objektum pozícióját PoseStamped formában kell előkészíteni a transzformáláshoz.
        
        pose_to_transform = PoseStamped()
        pose_to_transform.header.frame_id = "map" # Az objektumok forrás frame-je a 'map'
        pose_to_transform.header.stamp = msg.header.stamp # Az odometria időbélyegének használata
        pose_to_transform.pose = closest_marker_pose
        
        # 4. TF transzformáció végrehajtása (védve késés ellen)
        try:
            if not self.tf_buffer.can_transform(
                "agent1/base_link",
                pose_to_transform.header.frame_id,
                msg.header.stamp,
                Duration(seconds=0.5)
            ):
                self.get_logger().warn('TF nincs készen: map -> agent1/base_link')
                return

            transform = self.tf_buffer.lookup_transform(
                "agent1/base_link",
                pose_to_transform.header.frame_id, # 'map'
                msg.header.stamp,
                Duration(seconds=0.5)
            )

            # A PoseStamped üzenet transzformálása a cél frame-be
            transformed_pose = tf2_geometry_msgs.do_transform_pose(pose_to_transform, transform)
            transformed_pose.header.frame_id = "agent1/base_link"

            # 5. Publikálás
            self.pub_odom.publish(transformed_pose)
        except Exception as exc:
            self.get_logger().warn(f'TF hiba: {exc}')

    def timer_callback(self):
        marker_array = MarkerArray()
        self.markers = []
        now = self.get_clock().now().to_msg()
        
        for i in range(1, 1001):
            x = 10 * math.cos(i) + 0.1 * i - 10
            y = 10 / i - 0.5
            z = math.sqrt(x * x + y * y)

            marker = Marker()
            marker.header.frame_id = "map" # A kapott objektum lista az “map” frame-ban van értelmezve
            marker.header.stamp = now
            marker.ns = "generated_objects"
            marker.id = i            
            marker.type = marker.SPHERE # Gömb marker
            marker.action = marker.ADD # Hozzáadás/Frissítés

            # Pozíció beállítása
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = z
            marker.pose.orientation.w = 1.0 # Alapértelmezett, nincs forgatás

            # Méret beállítása: 15 cm átmérőjű gömbök legyenek
            marker.scale.x = 0.15
            marker.scale.y = 0.15
            marker.scale.z = 0.15

            # Szín beállítása: véletlenszerű szín
            marker.color.r = random.uniform(0.0, 1.0)
            marker.color.g = random.uniform(0.0, 1.0)
            marker.color.b = random.uniform(0.0, 1.0)
            marker.color.a = 1.0 # Teljesen átlátszatlan

            marker.lifetime = Duration(seconds=0.1).to_msg()

            self.markers.append(marker)

        # Az összes előre generált markert hozzáadjuk a MarkerArray-hez
        # Ezt deepcopy-val kellene megoldani, ha dinamikusan frissülne a Marker ID.
        # Mivel itt statikus, de RViz MarkerArray-ként várja, egyszerűen hozzáadjuk.
        marker_array.markers = self.markers
        # Fontos: Frissítjük a headert és az időbélyeget minden markerben
        # A MarkerArray-hez nem feltétlenül kell header, de minden markernek kell
        now = self.get_clock().now().to_msg()
        for i, marker in enumerate(marker_array.markers):
            # Az időbélyeg és a frame_id frissítése az aktuális időpontra és map-ra
            marker.header.stamp = now
            # Az ID-t frissíteni kell minden küldésnél, ha dinamikus (de itt fix).
            # Minden ID egyedi kell legyen a namespace-en belül.

        self.pub_obj.publish(marker_array)

        # Objektumok számának kiszámítása a második síknegyedben
        count = 0
        
        # Iterálunk az előre generált markerek listáján
        for marker in self.markers:
            x = marker.pose.position.x
            y = marker.pose.position.y
            
            # Feltétel: X pozitív ÉS Y pozitív (Második síknegyed)
            if x > 0.0 and y > 0.0:
                count += 1

        count_msg = Int32()
        count_msg.data = count
        
        self.pub_object_count.publish(count_msg)

def main(args=None):
    # ROS inicializálás.
    rclpy.init(args=args)
    
    # Node létrehozása.
    zh = ObjectPublisherNode()
    
    # Eseményciklus indítása.
    rclpy.spin(zh)
    
    # Erőforrások felszabadítása.
    zh.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()