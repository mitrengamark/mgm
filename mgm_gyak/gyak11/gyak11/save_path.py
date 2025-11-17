"""PathSavingNode node - Path mentése fájlba odometriából.

Magyarázat:
 - Feliratkozik az odometria topicra és minden új pozíciót ment fájlba.
 - Ritkítás: csak akkor ment, ha a távolság az előző mentetttől nagyobb, mint 'distance' paraméter.
 - TF transzformáció: ha az odometria nem "map" frame-ben van, transzformáljuk.
 - Quaternion → yaw konverzió (euler_from_quaternion), majd fokban.
 - Fájlba írás: x, y, z, yaw (CSV formátum, kerekítve).
"""

import rclpy
from rclpy.node import Node
import math

from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
import tf2_ros
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
from tf_transformations import euler_from_quaternion

class PathSavingNode(Node):

    def __init__(self):
        super().__init__('path_saving_node')

        # Paraméterek: odometria topic neve, fájlnév, minimális távolság.
        self.declare_parameter("topic_name", "/odom")
        topic_name = self.get_parameter("topic_name").value

        self.declare_parameter("text_file_name", "/positions.txt")
        text_file_name = self.get_parameter("text_file_name").value

        self.declare_parameter("distance", 0.1)  # Minimális távolság a mentések között.
        self.dist = self.get_parameter("distance").value

        # Előző mentett pozíció (kezdetben origó).
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        # TF Buffer és Listener (transzformációhoz, ha szükséges).
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)

        # Fájl megnyitása írásra (figyelem: felülírja a meglévőt!).
        self.file_writer = open(text_file_name, "w")

        # Subscriber: odometria fogadása.
        self.sub = self.create_subscription(Odometry, topic_name, self.odom_callback, 1)


    def odom_callback(self, msg:Odometry):
        # Távolság számítása az előző mentett pozíciótól.
        dx = self.x - msg.pose.pose.position.x
        dy = self.y - msg.pose.pose.position.y
        dz = self.z - msg.pose.pose.position.z
        distance = math.sqrt(dx**2 + dy**2 + dz**2)

        # Ha a távolság nagyobb, mint a küszöb, mentjük a pozíciót.
        if (distance > self.dist):

            # Aktuális póz lekérdezése.
            actual_pose = Pose()
            if msg.header.frame_id == "map":
                # Már "map" frame-ben van, nincs szükség transzformációra.
                actual_pose = msg.pose.pose

            else:
                # Transzformáció "map" frame-be (TF-fel).
                if self.tfBuffer.can_transform("map", msg.header.frame_id, rclpy.time.Time(), rclpy.duration.Duration(seconds=0.1)):
                    trans_2map = self.tfBuffer.lookup_transform("map", msg.header.frame_id, rclpy.time.Time())

                    actual_pose = do_transform_pose(msg.pose.pose, trans_2map)
                else:
                    self.get_logger().info("error in tf")
                    return

            # Yaw számítása: quaternion → Euler szögek (roll, pitch, yaw).
            q = [
                actual_pose.orientation.x,
                actual_pose.orientation.y,
                actual_pose.orientation.z,
                actual_pose.orientation.w
            ]
            yaw_radian = euler_from_quaternion(q)[2]
            yaw = yaw_radian / math.pi * 180.0  # Radiánból fokba.

            # Fájlba írás: x, y, z, yaw (kerekítve, CSV formátum).
            self.file_writer.write(str(round(actual_pose.position.x, 2)) + ', ')
            self.file_writer.write(str(round(actual_pose.position.y, 2)) + ', ')
            self.file_writer.write(str(round(actual_pose.position.z, 2)) + ', ')
            self.file_writer.write(str(round(yaw,4)))
            self.file_writer.write('\n')

            # Előző mentett pozíció frissítése.
            self.x = msg.pose.pose.position.x
            self.y = msg.pose.pose.position.y
            self.z = msg.pose.pose.position.z







def main(args=None):
    # ROS inicializálás.
    rclpy.init(args=args)
    
    # Node létrehozása.
    path_saving = PathSavingNode()
    
    # Eseményciklus indítása.
    rclpy.spin(path_saving)
    
    # Erőforrások felszabadítása.
    path_saving.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()