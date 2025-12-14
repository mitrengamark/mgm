import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import tf2_ros
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose_stamped

# MAGYAR KOMMENT: TF2 Pose átalakítás bemutató
# - A `PoseStamped` rendelkezik egy `header.frame_id` mezővel (itt: 'base_link').
# - A TF2 `Buffer` és `TransformListener` gondoskodik a transzformációk elérhetőségéről.
# - A `can_transform()` ellenőrzi, hogy van-e érvényes transzformáció két frame között.
# - A `lookup_transform()` lekéri a tényleges transzformációt (pl. 'map' <- 'base_link').
# - A `do_transform_pose_stamped()` a bemeneti pozíciót/kvaterniót átszámolja a cél frame-be.

class TF2PoseExample(Node):
    """Példa: PoseStamped átalakítása 'base_link'→'map' TF2-vel (MAGYARUL).

    Lépések:
    1) Összeállítunk egy helyi pozíciót 'base_link' frame-ben.
    2) Megnézzük, hogy létezik-e transzformáció 'map' és 'base_link' között.
    3) Ha igen, átszámoljuk a pozíciót 'map' frame-be és kiírjuk az eredményt.
    """
    def __init__(self):
        super().__init__('examples_tf2_pose')
        self.tf_buffer = tf2_ros.Buffer()  # TF adatok pufferelése
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)  # Listener tölti be a TF-et
        self.timer = self.create_timer(0.5, self.timer_cb)  # Időzítő: fél másodpercenként futtatjuk

    def timer_cb(self):
        # 1) Helyi (base_link) pozíció összeállítása
        pose_local = PoseStamped()
        pose_local.header.frame_id = 'base_link'
        pose_local.header.stamp = self.get_clock().now().to_msg()
        pose_local.pose.orientation.w = 1.0  # Egység kvaternió (nincs elfordulás)
        pose_local.pose.position.x = 1.0  # Példa pozíció X=1.0

        # 2) Ellenőrizzük, hogy rendelkezésre áll-e a 'map'←'base_link' transzformáció
        if self.tf_buffer.can_transform('map', 'base_link', rclpy.time.Time(), rclpy.duration.Duration(seconds=0.2)):
            # 3) Lekérjük a transzformációt és átalakítjuk a PoseStamped-et
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            pose_map = do_transform_pose_stamped(pose_local, t)
            self.get_logger().info(f"Átalakított pozíció (map): x={pose_map.pose.position.x:.2f}")
        else:
            # Ha nincs még TF (például a broadcaster nem indult el), jelezzük
            self.get_logger().warn('TF nem elérhető: map<-base_link')


def main():
    rclpy.init()
    n = TF2PoseExample()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()
