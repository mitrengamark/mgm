import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
import math
from geometry_msgs.msg import PoseArray, Pose

"""Készítsen egy node-t, mely feliratkozik egy 
„/agent1/scan” nevű „sensor_msgs/LaserScan” típusú topicra, 
melyből vizsgálja meg a +/-5°-os tartományban van-e objektum 
a szenzor érzékelési távolság tartománya szerint. 
Ha van, akkor a legközelebbi objektumtól való távolságot 
küldje ki egy „std_msgs/Float64” típusú üzenetben.

A vizsgált érzékelési tartományban lévő pontokat 
számolja át derékszögi koordináta rendszerben értelmezett 
koordinátákra és kommunikálja ki azokat „geometry_msgs/PoseArray”
típusú üzenetként.
"""


class Node1(Node):
    def __init__(self):
        super().__init__('node1')

        self.sub_scan = self.create_subscription(LaserScan, '/agent1/scan', self.scan_callback, 1)

        self.pub_dist = self.create_publisher(Float64, '/closest_distance', 1)

        self.pub_coords = self.create_publisher(PoseArray, '/detected_points', 1)

    def scan_callback(self, msg: LaserScan):
        # Tartomány szűrés és legközelebbi objektum keresése
        # Szög intervallum: +/-5° = +/- (5 * pi/180) rad (közép: 0 rad)
        # Minták és tartomány ellenőrzés mintája: gyak9/gyak9/scan_tf.py (50-74. sor)
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment
        range_min = msg.range_min
        range_max = msg.range_max

        target_min = -5.0 * math.pi / 180.0
        target_max =  5.0 * math.pi / 180.0

        closest = None
        detected_poses = []  # Derékszögi koordináták tárolása

        for i, r in enumerate(msg.ranges):
            angle = angle_min + i * angle_inc
            if target_min <= angle <= target_max:
                if range_min < r < range_max:
                    # Legközelebbi pont keresése
                    if closest is None or r < closest:
                        closest = r
                    
                    # Poláris → Derékszögi konverzió (gyak9/scan_tf.py: 66-70. sorok alapján)
                    x = r * math.cos(angle)
                    y = r * math.sin(angle)
                    z = 0.0
                    
                    # Pose létrehozása (templates/examples_markers.py: 23-24. sorok mintájára)
                    pose = Pose()
                    pose.position.x = x
                    pose.position.y = y
                    pose.position.z = z
                    pose.orientation.w = 1.0  # Nincs rotáció
                    detected_poses.append(pose)

        # Ha találtunk objektumot, publikáljuk a legkisebb távolságot Float64-ben
        if closest is not None:
            dist_msg = Float64()
            dist_msg.data = closest
            self.pub_dist.publish(dist_msg)
            self.get_logger().info(f"Closest object within +/-5°: {closest:.3f} m")
        
        # PoseArray publikálása derékszögi koordinátákkal (gyak3/path.py: 58-60. sorok mintájára)
        if detected_poses:
            pose_array = PoseArray()
            pose_array.header.stamp = msg.header.stamp
            pose_array.header.frame_id = msg.header.frame_id
            pose_array.poses = detected_poses
            self.pub_coords.publish(pose_array)
            self.get_logger().info(f"Published {len(detected_poses)} detected points")

def main(args=None):
    # ROS inicializálás.
    rclpy.init(args=args)
    
    # Node létrehozása.
    proba3 = Node1()
    
    # Eseményciklus indítása.
    rclpy.spin(proba3)
    
    # Erőforrások felszabadítása.
    proba3.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()