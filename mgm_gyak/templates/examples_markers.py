import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.duration import Duration

class MarkerExamples(Node):
    """Example: Publish a sphere Marker and a MarkerArray of spheres."""
    def __init__(self):
        super().__init__('examples_markers')
        self.pub_marker = self.create_publisher(Marker, '/marker', 1)  # Publikálás egy Marker-nek
        self.pub_array = self.create_publisher(MarkerArray, '/markers', 1)  # Publikálás MarkerArray-nek
        self.timer = self.create_timer(1.0, self.timer_cb)

    def timer_cb(self):
        # Single sphere Marker
        m = Marker()
        m.header.frame_id = 'map'  # Frame ID beállítása
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'single'
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.pose.position.x = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.1
        m.color.a = 1.0
        m.color.r = 1.0
        self.pub_marker.publish(m)

        # MarkerArray of spheres
        arr = MarkerArray()
        for i in range(3):
            mi = Marker()
            mi.header = m.header
            mi.ns = 'array'
            mi.id = i
            mi.type = Marker.SPHERE
            mi.action = Marker.ADD
            mi.pose.orientation.w = 1.0
            mi.pose.position.x = 0.5 * i
            mi.scale.x = mi.scale.y = mi.scale.z = 0.1
            mi.color.a = 1.0
            mi.color.g = 1.0
            mi.lifetime = Duration(seconds=0.5).to_msg()
            arr.markers.append(mi)
        self.pub_array.publish(arr)
        self.get_logger().info('MarkerArray publikálva')


def main():
    rclpy.init()
    n = MarkerExamples()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()
