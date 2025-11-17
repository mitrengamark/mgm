"""VizPub node - Vizualizációs markerek publikálása.

Magyarázat:
 - Feliratkozik az odometriára és minden üzenetből több Marker-t hoz létre.
 - Marker típusok: CUBE (aktuális pozíció), SPHERE (path), CUBE (bounding box).
 - MarkerArray: több marker egyidejű publikálása egyetlen üzenetben.
 - Lifetime: marker meddig látható RViz-ben (0.1 sec).
 - Path: korábbi pózokat gyűjti, ritkítva (30-as lépésekkel) megjelenítve.
 - Bounding box: megkeresi az eddig bejárt pontok x,y min/max értékeit.
"""

import rclpy
from rclpy.node import Node
import random
import copy

from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray, Marker


class VizPub(Node):
    def __init__(self):
        super().__init__('test_viz')

        # Paraméter: odometria topic neve.
        odom_topic_name = self.declare_parameter('odom_topic_name', '/odom').value
        
        # Subscriber: odometria fogadása.
        self.sub = self.create_subscription(Odometry, odom_topic_name, self.callback_odom, 1)
        
        # Publisher: MarkerArray publikálása RViz vizualizációhoz.
        self.pub = self.create_publisher(MarkerArray, "/viz", 1)
        
        # Lista a korábbi pózok tárolására (path építéshez).
        self.pose_list = []

    def callback_odom(self, msg: Odometry):
        # MarkerArray: több marker tárolására szolgáló tömb.
        marker_array = MarkerArray()

        # Marker: egy vizuális objektum (CUBE típus).
        marker = Marker()
        marker.header = msg.header  # Időbélyeg és frame.
        marker.ns = "cube"  # Namespace: markerek csoportosítása.
        marker.id = 0  # Egyedi azonosító a namespace-en belül.
        marker.type = Marker.CUBE  # Típus: kocka.
        marker.action = Marker.ADD  # Művelet: hozzáadás/frissítés.

        # Pozíció és orientáció: az aktuális odometria pózból.
        marker.pose = msg.pose.pose

        # Méret: x, y, z irányban (méterben).
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.4

        # Szín: RGBA formátum (alpha=1.0 teljesen átlátszatlan, kék szín).
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0

        # Lifetime: marker meddig látható (0.1 sec után eltűnik, ha nincs frissítés).
        marker.lifetime = rclpy.duration.Duration(seconds=0.1).to_msg()
        
        # Marker hozzáadása a tömbőhöz.
        marker_array.markers.append(marker)

        # Path markerek hozzáadása (korábbi pózok vizualizációja).
        self.path_publisher(msg, marker_array)
        
        # Bounding box marker hozzáadása (bejárt terület határai).
        self.rectangle_publisher(marker_array)

        # Teljes MarkerArray publikálása.
        self.pub.publish(marker_array)

    def path_publisher(self, msg: Odometry, marker_array_: MarkerArray):
        # Aktuális póz hozzáadása a listához (path építés).
        self.pose_list.append(msg.pose.pose)
        
        # Path marker: SPHERE típus (gömb).
        marker_path = Marker()

        marker_path.header = msg.header

        marker_path.ns = "path"  # Namespace: path markerek.

        marker_path.type = Marker.SPHERE
        marker_path.action = Marker.ADD

        # Kisebb méret (0.1 m átmérő).
        marker_path.scale.x = 0.1
        marker_path.scale.y = 0.1
        marker_path.scale.z = 0.1

        marker_path.lifetime = rclpy.duration.Duration(seconds=0.1).to_msg()

        # Ritkítás: csak minden 30. pózhoz készítünk markert (teljesítményoptimalizálás).
        for i in range(0, len(self.pose_list), 30):
            marker_path.id = i  # Egyedi ID minden markerhez.
            marker_path.pose = self.pose_list[i]

            # Véletlen színek (változatos megjelenés).
            marker_path.color.a = random.uniform(0.2, 1.0)
            marker_path.color.r = random.uniform(0.0, 1.0)
            marker_path.color.g = random.uniform(0.0, 1.0)
            marker_path.color.b = random.uniform(0.0, 1.0)

            # Deep copy: új objektum létrehozása (különben minden marker ugyanaz lenne).
            marker_array_.markers.append(copy.deepcopy(marker_path))

    def rectangle_publisher(self, marker_array_: MarkerArray):
        # Inicializálás: nagy/kis kezdőértékek a min/max kereséshez.
        x_min = 999.0
        x_max = -999.0
        y_min = 999.0
        y_max = -999.0

        # Végigmegyünk az összes korábbi pózón, megkeresve a szélsőértékeket.
        for i in range(0,len(self.pose_list)-1):
            i_pose = self.pose_list[i]

            if (x_min > i_pose.position.x):
                x_min = i_pose.position.x

            if (x_max < i_pose.position.x):
                x_max = i_pose.position.x
    
    
            if (y_min > i_pose.position.y):
                y_min = i_pose.position.y

            if (y_max < i_pose.position.y):
                y_max = i_pose.position.y

        self.get_logger().info(f"x_min: {x_min} x_max: {x_max}")

        # Bounding box marker: CUBE típus, félátlátszó piros téglalap.
        marker_rectangle = Marker()

        marker_rectangle.header.frame_id = "odom"
        marker_rectangle.header.stamp = self.get_clock().now().to_msg()

        marker_rectangle.ns = "rect"
        marker_rectangle.id = 0
        marker_rectangle.type = Marker.CUBE
        marker_rectangle.action = Marker.ADD

        # Szín: piros, alpha=0.2 (átlátszó).
        marker_rectangle.color.a = 0.2
        marker_rectangle.color.r = 1.0
        marker_rectangle.color.b = 0.0
        marker_rectangle.color.g = 0.0

        # Pozíció: a bounding box középpontja (min+max)/2.
        marker_rectangle.pose.position.x = (x_min+x_max)/2.0
        marker_rectangle.pose.position.y = (y_min+y_max)/2.0
        marker_rectangle.pose.orientation.w = 1.0  # Nincs forgatás (quaternion).
        
        # Méret: bounding box szélessége és magassága.
        marker_rectangle.scale.x = abs(x_max - x_min)
        marker_rectangle.scale.y = abs(y_max - y_min)
        marker_rectangle.scale.z = 0.05  # Vékony lap (síkbeli megjelenítéshez).

        marker_array_.markers.append(marker_rectangle)


def main(args=None):
    # ROS inicializálás.
    rclpy.init(args=args)
    
    # Node létrehozása.
    viz_publisher = VizPub()
    
    # Eseményciklus indítása.
    rclpy.spin(viz_publisher)
    
    # Erőforrások felszabadítása.
    viz_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()