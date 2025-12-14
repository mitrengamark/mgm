import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler, euler_from_quaternion

# MAGYAR KOMMENT: Odometry + kvaternió példa
# - A kvaterniót `quaternion_from_euler(roll, pitch, yaw)` segítségével állítjuk elő.
# - Visszaalakításra `euler_from_quaternion((x, y, z, w))` használható (itt a yaw-t ellenőrzéshez kiírjuk).

class OdometryQuat(Node):
    """Példa: Odometry publikálása kvaternióval; yaw kiolvasása (MAGYARUL)."""
    def __init__(self):
        super().__init__('examples_odometry_quat')
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)  # Odometry publikáló
        self.declare_parameter('yaw_rad', 0.5)          # Bemenő yaw radiánban (példa)
        self.declare_parameter('target_yaw_rad', 0.0)   # Cél yaw, hogy hibát számoljunk
        self.timer = self.create_timer(0.5, self.timer_cb)  # Időzítő: fél másodpercenként

    def timer_cb(self):
        # Odometry összeállítása: frame-ek és pozíció megadása
        odom = Odometry()
        odom.header.frame_id = 'odom'  # szülő koordináta-rendszer
        odom.child_frame_id = 'base_link'  # gyermek (robot törzs)
        odom.pose.pose.position.x = 1.0

        # Kvaternió létrehozása paraméterezett yaw alapján
        yaw_in = float(self.get_parameter('yaw_rad').value)
        q = quaternion_from_euler(0.0, 0.0, yaw_in)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        self.pub_odom.publish(odom)

        # Kvaternió visszaalakítása Eulerre: yaw ellenőrzés céljából
        q_tuple = (
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w,
        )
        yaw = euler_from_quaternion(q_tuple)[2]

        # Példa: yaw hiba számítás egy cél yaw-hoz (vezérléshez felhasználható)
        target_yaw = float(self.get_parameter('target_yaw_rad').value)
        yaw_error = target_yaw - yaw

        # Itt (kommentben) jöhetne egy egyszerű P-vezérlés vagy döntés logika
        # pl.: cmd.angular.z = k_p * yaw_error

        self.get_logger().info(
            f"Yaw = {yaw:.2f} rad | Cél yaw = {target_yaw:.2f} rad | Hiba = {yaw_error:.2f} rad")


def main():
    rclpy.init()
    n = OdometryQuat()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()
