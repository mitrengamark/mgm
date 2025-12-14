import rclpy
from rclpy.node import Node
import math
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker
from std_msgs.msg import Header

"""Készítsen egy node-t, mely feliratkozik egy
 „/agent1/odom/ground_truth” nevű „nav_msgs/Odometry”
 típusú topicra, melyből vizsgálja az 'x' és 'y' 
 irányú pozíció koordinátát megcserélve 
 egy „visualization_msgs/Marker” üzenetet küldjön ki. 
 Az üzenet egy 10x10x20 cm-es téglatestet jelenítsen, 
 melynek hosszú oldala ugyan abba az irányba mutat, mint a jármű.
 
 Egy  0.2 Hz-es timer segítségével küldje ki, 
 az utolsó „Odometry” üzenet időbélyegét egy „std_msgs/Header” 
 típusú üzenetben. 
 Készítsen egy launch file-t, mely elindítja a node-t 
 és egy rviz-t, melyben meg lehet jeleníteni a pozíciót 
 és a kiküldött téglatestet.
 """


class Node1(Node):
    def __init__(self):
        super().__init__('node1')

        # Utolsó odometry időbélyeg tárolása (gyak9/scan_tf.py: 22. sor mintájára)
        self.last_odom_header = Header()

        self.sub_odom = self.create_subscription(Odometry, '/agent1/odom/ground_truth', self.odom_callback, 1)

        # Publisher: Marker vizualizáció (gyak12/node2.py: 37. sor alapján)
        self.pub_marker = self.create_publisher(Marker, '/swapped_marker', 1)

        self.timer = self.create_timer(5.0, self.timer_callback)  # 0.2 Hz timer

        self.pub_last_odom = self.create_publisher(Header, '/last_odom_header', 1)

    def timer_callback(self):
        # Timer callback: utolsó odometry időbélyeg publikálása (gyak2/publisher.py: 31-41. sorok alapján)
        # gyak9/scan_tf.py: 76-84. sorok mintájára
        self.pub_last_odom.publish(self.last_odom_header)
        self.get_logger().info(f'Published last odom header timestamp: {self.last_odom_header.stamp.sec}.{self.last_odom_header.stamp.nanosec}')

    def odom_callback(self, msg: Odometry):
        # Odometry header mentése (gyak9/scan_tf.py: 84-86. sorok alapján)
        self.last_odom_header = msg.header

        x = msg.pose.pose.position.y
        y = msg.pose.pose.position.x
        z = msg.pose.pose.position.z

        marker = Marker()

        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "swapped_marker"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        
        # Orientáció másolása: jármű irányába mutat (gyak10/pure_pursuit.py: 174. sor mintájára)
        marker.pose.orientation = msg.pose.pose.orientation

        marker.scale.x = 0.1  # 10 cm
        marker.scale.y = 0.1  # 10 cm
        marker.scale.z = 0.2  # 20 cm (hosszú oldal)
        
        # Szín és átlátszóság (gyak12/node2.py: 110-113. sorok alapján)
        marker.color.a = 1.0  # Teljes átlátszatlanság
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0  # Kék szín
        
        # Lifetime: marker élettartama (gyak10/pure_pursuit.py: 182. sor alapján)
        marker.lifetime = rclpy.duration.Duration(seconds=0.1).to_msg()
        
        # Marker publikálása (gyak10/pure_pursuit.py: 186. sor mintájára)
        self.pub_marker.publish(marker)


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