# Példa: nav_msgs/Path publikálása és feldolgozása
# Cél: megmutatni, hogyan épül fel egy Path és hogyan lehet feliratkozni rá.

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class PathPeldaPublisher(Node):
    def __init__(self):
        super().__init__('path_pelda_pub')
        self.pub = self.create_publisher(Path, '/path', 10)
        self.timer = self.create_timer(1.0, self.timer_cb)
        self.t = 0.0

    def timer_cb(self):
        msg = Path()
        msg.header.frame_id = 'map'
        # Egyszerű vonal: 5 pont
        for i in range(5):
            p = PoseStamped()
            p.header.frame_id = 'map'
            p.pose.position.x = 0.2 * i
            p.pose.position.y = 0.0
            p.pose.orientation.w = 1.0
            msg.poses.append(p)
        self.pub.publish(msg)
    # MAGYAR KOMMENT: Path publikálás és előfizetés
    # - A Path egy `PoseStamped` listát tartalmaz (útvonal pontjai).
    # - Publikáláskor beállítjuk a `header.frame_id`-t és feltöltjük a `poses` listát.

class PathPeldaSubscriber(Node):
    def __init__(self):
        super().__init__('path_pelda_sub')
        self.sub = self.create_subscription(Path, '/path', self.cb, 10)
    def cb(self, msg: Path):
        self.get_logger().info(f'Path pontok száma: {len(msg.poses)}')

# Sablon: nem kell futtatni a ZH-ban.
