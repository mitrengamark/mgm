"""ZhPub node - Odometria alapján távolság számolás és publikálás.

Magyarázat:
 - Feliratkozik az /odom topicra, kiszámolja a bejárt távolságot.
 - Ha a távolság egész része megváltozik (pl. 1.9 → 2.1), publikálja az odometriát.
 - Publikálja a teljes bejárt távolságot (Float64) a /elmozdulas topicra.
 - Simán használható robot mozgásának nyomon követésére.
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

class ZhPub(Node):
    def __init__(self):
        super().__init__('zh_node')

        # Subscriber: odometria fogadása.
        self.subscription = self.create_subscription(Odometry,'/odom',self.callback_odom,1)
        
        # Publisher: odometria publikálása (feltételes).
        self.publisher_odom = self.create_publisher(Odometry, '/poz', 1)
        
        # Publisher: teljes távolság publikálása.
        self.publisher_dist = self.create_publisher(Float64, '/elmozdulas', 1)

        # Távolság tárolása (Float64 üzenet).
        self.dist_msg = Float64()
        
        # Utolsó mentett pozíció (kezdeti érték: origó).
        self.last_msg = Point()
        
        self.get_logger().info('ZH Publisher node has been started')

    def callback_odom(self, msg: Odometry):
        # Elmozdulas számítása: Euclidean távolság az előző pozíciótól.
        elmozdulas = math.sqrt(
            (msg.pose.pose.position.x - self.last_msg.x)**2 + 
            (msg.pose.pose.position.y - self.last_msg.y)**2
        )
        
        # Feltételes publikálás: ha a távolság egész része növekszik.
        # Például: floor(1.9) = 1, floor(2.1) = 2 → publikál.
        if math.floor(self.dist_msg.data) < math.floor(self.dist_msg.data + elmozdulas):
            self.publisher_odom.publish(msg)
        
        # Teljes távolság frissítése és publikálása.
        self.dist_msg.data += elmozdulas
        self.publisher_dist.publish(self.dist_msg)

        # Utolsó pozíció frissítése.
        self.last_msg = msg.pose.pose.position



def main(args=None):
    # ROS inicializálás.
    rclpy.init(args=args)
    
    # Node létrehozása.
    zh_pub = ZhPub()
    
    # Eseményciklus indítása.
    rclpy.spin(zh_pub)
    
    # Erőforrások felszabadítása.
    zh_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()