# Példa: Útvonal (Path) CSV beolvasása és publikálása
# Megjegyzés: Tanulási sablon; nem kötelező futtatni.

import csv
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class CSVPathPelda(Node):
    def __init__(self):
        super().__init__('csv_path_pelda')
        self.pub = self.create_publisher(Path, '/path', 10)
        self.declare_parameter('csv_file', 'positions.csv')
        self.declare_parameter('frame_id', 'map')
        self.timer = self.create_timer(2.0, self.publish_path)
        self.path = self.load_csv(self.get_parameter('csv_file').value,
                                  self.get_parameter('frame_id').value)

    def load_csv(self, filename, frame_id):
        path = Path()
        path.header.frame_id = frame_id
        try:
            with open(filename, newline='') as f:
                reader = csv.reader(f)
                for row in reader:
                    # Sor formátum: x,y (opcionálisan yaw)
                    if not row:
                        continue
                    x = float(row[0])
                    y = float(row[1])
                    pose = PoseStamped()
                    pose.header.frame_id = frame_id
                    pose.pose.position.x = x
                    pose.pose.position.y = y
                    pose.pose.orientation.w = 1.0
                    path.poses.append(pose)
        except FileNotFoundError:
            self.get_logger().warn(f'CSV nem található: {filename}')
        except Exception as e:
            self.get_logger().error(f'Hiba CSV beolvasás közben: {e}')
        return path

    def publish_path(self):
        self.pub.publish(self.path)
        self.get_logger().info(f'Publikált path pontok: {len(self.path.poses)}')

# Sablon: nem kell futtatni a ZH-ban.
 # MAGYAR KOMMENT: CSV→Path
 # - CSV formátum: soronként `x,y` (opcionálisan `yaw`).
 # - Hibakezelés: ha nincs fájl, figyelmeztetünk; egyéb hibát logolunk.
 # - A path `frame_id`-je a paraméterből jön (alapértelmezés: 'map').
