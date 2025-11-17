"""PathPublisher node - Path betöltése fájlból és publikálása.

Magyarázat:
 - Beolvas egy text fájlt (positions.txt), amely soronként tárolja a pozíciókat: x, y, z, yaw.
 - Path üzenet létrehozása: PoseStamped elemekből álló lista.
 - Yaw (fokban) → quaternion konverzió (quaternion_from_euler).
 - Marker vizualizáció: x_min, y_min és x_max, y_max pontok (piros gömbök).
 - TF transzformáció: opcionális (ha frame_name != "map").
 - Publikálási frekvencia: publish_rate paraméterrel beállítható.
"""

import math
import copy
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import MarkerArray, Marker

import tf2_ros
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose_stamped
from tf_transformations import quaternion_from_euler

class PathPublisher(Node):

    def __init__(self):
        super().__init__('path_publisher')

        # Paraméterek: publikálási frekvencia, frame név, fájlnév.
        self.declare_parameter('publish_rate', 1.0)
        publish_rate = self.get_parameter('publish_rate').value
        timer_period = 1/publish_rate  # seconds

        self.declare_parameter('frame_name', 'map')
        self.frame_name = self.get_parameter('frame_name').value

        self.declare_parameter('text_file_name', '/workspace/src/ros2_mgm/applications/mgm_gyak/gyak11/param/positions.txt')
        self.text_file_name = self.get_parameter('text_file_name').value

        # TF Buffer és Listener (opcionális transzformációhoz).
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)

        # Publisher: path és vizualizációs markerek.
        self.publisher = self.create_publisher(Path, "/path", 10)
        self.pub_viz = self.create_publisher(MarkerArray, "/viz", 10)
        self.timer = self.create_timer(timer_period, self.timer_callback)
  
        # Fájl beolvasása (positions.txt).
        self.file_reader = open(self.text_file_name, 'r')
        self.path = self.file_reader.readlines()  # Sorok listája.
        self.file_reader.close()

        # Adat feldolgozás: minden sor négy értéket tartalmaz (x, y, z, yaw).
        self.path_values = []
        for x in range(len(self.path)):
            for value in self.path[x].split(', '):
                self.path_values.append(float(value))

        self.len_values = int(len(self.path_values)/4)  # Hány póz van.

        # Path üzenet létrehozása.
        self.msg = Path()
        self.msg.header.frame_id = "map"
        for i in range(self.len_values):

            # Yaw (fokokban) → quaternion konverzió (radiánra kell konvertálni először).
            q = quaternion_from_euler(0,0,(self.path_values[i*4+3] / 180.0 * math.pi))

            # PoseStamped üzenet létrehozása.
            msg_pose = PoseStamped()
            msg_pose.pose.position.x =self.path_values[i*4]
            msg_pose.pose.position.y = self.path_values[i*4+1]
            msg_pose.pose.position.z = self.path_values[i*4+2]
            msg_pose.pose.orientation.x = q[0]
            msg_pose.pose.orientation.y = q[1]
            msg_pose.pose.orientation.z = q[2]
            msg_pose.pose.orientation.w = q[3]

            # Póz hozzáadása a path-hoz.
            self.msg.poses.append(msg_pose)

        # Bounding box kiszámítása (x,y szélsőértékek).
        x_min = 9999.0
        x_max = -9999.0
        y_min = 9999.0
        y_max = -9999.0
        for i_pose in self.msg.poses:

            x = i_pose.pose.position.x
            y = i_pose.pose.position.y

            if x < x_min:
                x_min = x
            if x > x_max:
                x_max = x

            if y < y_min:
                y_min = y
            if y > y_max:
                y_max = y

        # Marker array létrehozása (min/max pontok vizualizációja).
        self.mark_array = MarkerArray()
        mark = Marker()
        mark.header.frame_id = "map"
        
        mark.ns = "sphere"
        mark.type = Marker.SPHERE
        mark.action = Marker.ADD

        mark.scale.x = 0.1
        mark.scale.y = 0.1
        mark.scale.z = 0.1

        mark.color.a = 1.0
        mark.color.r = 1.0  # Piros gömb.
        mark.color.g = 0.0
        mark.color.b = 0.0

        mark.lifetime = rclpy.duration.Duration(seconds=1.1).to_msg()

        # Első marker: (x_min, y_min).
        mark.id = 0
        mark.pose.position.x = x_min
        mark.pose.position.y = y_min
        mark.pose.orientation.w = 1.0

        self.mark_array.markers.append(copy.deepcopy(mark))

        # Második marker: (x_max, y_max).
        mark.id = 1
        mark.pose.position.x = x_max
        mark.pose.position.y = y_max
        mark.pose.orientation.w = 1.0

        self.mark_array.markers.append(copy.deepcopy(mark))



    def timer_callback(self):
        # Ha a frame "map", egyszerűen publikáljuk az eredeti path-ot.
        if (self.frame_name == "map"):
            self.msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(self.msg)

        else:
            # Más frame-be transzformáljuk a path-ot (TF-fel).
            if self.tfBuffer.can_transform("map", self.frame_name, rclpy.time.Time(), rclpy.duration.Duration(seconds=0.1)):
                trans_2frame = self.tfBuffer.lookup_transform("map", self.frame_name, rclpy.time.Time())
                
                # Minden pózt transzformálunk.
                for i in range(len(self.msg.poses)):
                    self.msg.poses[i] = do_transform_pose_stamped(self.msg.poses[i], trans_2frame)
                    self.msg.poses[i].header = self.frame_name

                self.msg.header.stamp = self.get_clock().now().to_msg()
                self.msg.header.frame_id = self.frame_name
                self.publisher.publish(self.msg)

            else:
                return
        
        # Marker array publikálása (min/max pontok).
        self.pub_viz.publish(self.mark_array)


def main(args=None):
    # ROS inicializálás.
    rclpy.init(args=args)
    
    # Node létrehozása.
    path_publisher = PathPublisher()
    
    # Eseményciklus indítása.
    rclpy.spin(path_publisher)
    
    # Erőforrások felszabadítása.
    path_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()