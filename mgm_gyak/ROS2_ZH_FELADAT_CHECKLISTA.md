# ROS 2 ZH/Feladat/Gyakorlat – Kötelező elemek és rövid minták

Cél: egyetlen, rövid, offline használható puskában összefoglalni mindazt, aminek gyakorlatilag minden ROS2-es feladatban benne kell lennie. Az elnevezések (node név, topic név, frame nevek) változhatnak, de a minták és a lépések azonosak.

---

## Alap Követelmények
- Node létrehozása `rclpy`-vel, tiszta belépési ponttal (`main()` + `if __name__ == '__main__':`).
- Publisher(ek), Subscriber(ek) és/vagy Timer(ek) létrehozása egyértelmű QoS-sal.
- Paraméterek: minden változtatható beállítás legyen paraméter (alapértékkel).
- Konzisztens topic és frame elnevezések (snake_case; `map/odom/base_link` TF konvenciók).
- Üzeneteknél `header.stamp` és `header.frame_id` helyes kezelése.
- TF használat: ha koordinátát alakítasz, legyen TF Buffer + Listener, és ellenőrizd a `can_transform`-ot.
- Naplózás: értelmes `get_logger().info()/warn()/error()` hívások fő lépéseknél és hibaágban.
- RViz vizualizációhoz Marker/MarkerArray (ha a feladat megjelenítést kér).
- Minimális hibakezelés és ellenőrzések (nulla/hossz-ellenőrzés, üres lista, időtúllépés TF-nél, stb.).

Példák a repóban:
- Publisher/Subscriber alap: `gyak2/gyak2/publisher.py`, `gyak2/gyak2/subscriber.py`
- Paraméterek és Path: `gyak3/gyak3/path.py`
- RViz markerek: `gyak5/gyak5/test_viz.py`
- LaserScan → PointCloud2: `gyak6/gyak6/test_scan.py`
- Robot szimuláció + PID: `gyak7/gyak7/diff_robot.py`, `gyak7/gyak7/control.py`
- TF használat: `gyak8/gyak8/test_tf.py`
- Multi-robot + PointCloud merge: `gyak9/gyak9/scan_tf.py`
- Ackermann + Pure Pursuit: `gyak10/gyak10/ack_robot.py`, `gyak10/gyak10/pure_pursuit.py`
- Path mentés/betöltés: `gyak11/gyak11/save_path.py`, `gyak11/gyak11/pub_path.py`
- ZH minták: `gyak12/gyak12/node1.py`, `gyak12/gyak12/node2.py`
---

## Minimális Node Váz
```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        # Paraméterek
        self.declare_parameter('rate_hz', 5.0)
        rate = float(self.get_parameter('rate_hz').value)
        # Timer
        self.timer = self.create_timer(1.0 / max(rate, 0.1), self.on_timer)

    def on_timer(self):
        self.get_logger().info('Tick')

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Példa fájlok:
- `gyak2/gyak2/publisher.py`
- `gyak2/gyak2/subscriber.py`
---

## Publisher / Subscriber Minta
```python
from std_msgs.msg import String

class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.pub = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(1.0, self.publish_once)

    def publish_once(self):
        msg = String(); msg.data = 'hello'
        self.pub.publish(msg)

class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        self.sub = self.create_subscription(String, 'chatter', self.on_msg, 10)

    def on_msg(self, msg: String):
        self.get_logger().info(f'got: {msg.data}')
```

---

## Paraméterek – Kötelező Minta
```python
self.declare_parameter('max_speed', 0.2)
self.max_speed = float(self.get_parameter('max_speed').value)
```

- Minden konstans, amit hangolhatsz (sebességek, távolság, lookahead, queue), legyen paraméter.

Példa fájlok:
- `gyak3/gyak3/path.py`
- `gyak7/gyak7/control.py`
- `gyak10/gyak10/ack_robot.py`, `gyak10/gyak10/pure_pursuit.py`
- `gyak11/gyak11/pub_path.py`
---

## Timer és Időkezelés
- Használj `create_timer(period, callback)`-et fix frekvenciához.
- Időbélyeghez: `self.get_clock().now().to_msg()`.

```python
self.timer = self.create_timer(0.1, self.step)  # 10 Hz
```

Példa fájlok:
- `gyak5/gyak5/test_viz.py`
- `gyak7/gyak7/diff_robot.py`
- `gyak10/gyak10/pure_pursuit.py`
---

## Üzenetek és Header Példák
```python
from std_msgs.msg import Header
hdr = Header()
hdr.stamp = self.get_clock().now().to_msg()
hdr.frame_id = 'map'
```
- A vizualizációkhoz és szenzoradatokhoz a frame következetes beállítása kritikus.

Példa fájlok:
- `gyak5/gyak5/test_viz.py`
- `gyak10/gyak10/ack_robot.py`
- `gyak11/gyak11/pub_path.py`
---

## TF – Transzformációk Alapminta
```python
import tf2_ros
from geometry_msgs.msg import PoseStamped
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose_stamped

self.tf_buffer = tf2_ros.Buffer()
self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

if self.tf_buffer.can_transform('map', 'base_link', rclpy.time.Time(), rclpy.duration.Duration(seconds=0.2)):
    T = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
    pose_bl = PoseStamped(); pose_bl.pose.orientation.w = 1.0
    pose_map = do_transform_pose_stamped(pose_bl, T)
```

- Ellenőrizd mindig a `can_transform`-ot, adj rövid timeoutot.

Példa fájlok:
- `gyak8/gyak8/test_tf.py`
- `gyak9/gyak9/scan_tf.py`
- `gyak10/gyak10/pure_pursuit.py`
- `gyak11/gyak11/save_path.py`, `gyak11/gyak11/pub_path.py`
- `gyak12/gyak12/node2.py`
---

## Marker / MarkerArray – RViz-hez
```python
from visualization_msgs.msg import Marker, MarkerArray
ma = MarkerArray()
m = Marker(); m.header.frame_id = 'map'; m.type = Marker.SPHERE; m.action = Marker.ADD
m.scale.x = m.scale.y = m.scale.z = 0.1
m.color.a = 1.0; m.color.r = 1.0
ma.markers.append(m)
self.pub_viz.publish(ma)
```

Példa fájlok:
- `gyak5/gyak5/test_viz.py`
- `gyak10/gyak10/pure_pursuit.py`
- `gyak12/gyak12/node2.py`
---

## LaserScan → Pontok → PointCloud2
```python
import math
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2 as pcl2

pts = []
for i, r in enumerate(scan.ranges):
    if scan.range_min < r < scan.range_max:
        ang = scan.angle_min + i * scan.angle_increment
        pts.append([r * math.cos(ang), r * math.sin(ang), 0.0])
cloud = pcl2.create_cloud_xyz32(scan.header, pts)
self.pub_cloud.publish(cloud)
```

Példa fájlok:
- `gyak6/gyak6/test_scan.py`
- `gyak9/gyak9/scan_tf.py`
- `gyak12/gyak12/node2.py`
---

## Path Kezelés (Odomból)
```python
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

self.path = Path(); self.path.header.frame_id = 'map'
pose = PoseStamped(); pose.header = odom.header; pose.pose = odom.pose.pose
self.path.poses.append(pose)
self.path_pub.publish(self.path)
```

- Túl hosszú path esetén régi elemek ritkítása (pl. lista elejéről törlés).

Példa fájlok:
- `gyak3/gyak3/path.py`
- `gyak11/gyak11/pub_path.py`
---

## Fájl I/O – Mentés és Betöltés
- Mentés (odom → txt): x, y, z, yaw_deg
- Betöltés (txt → Path): soronként 4 érték feldolgozása

```python
# Írás
with open('positions.txt', 'w') as f:
    f.write(f"{x:.2f}, {y:.2f}, {z:.2f}, {yaw_deg:.2f}\n")

# Olvasás és path építés
vals = []
for line in open('positions.txt'):
    vals.extend([float(v) for v in line.split(', ')])
```

Példa fájlok:
- `gyak11/gyak11/save_path.py` (mentés)
- `gyak11/gyak11/pub_path.py` (betöltés)
---

## Launch (XML) – Minimál
```xml
<launch>
  <node pkg="mypkg" exec="my_node" name="my_node" output="screen">
    <param name="rate_hz" value="5.0"/>
  </node>
</launch>
```

Példa fájlok:
- `gyak3/launch/gyak3.launch.xml`
- `gyak4/launch/gyak4.launch.xml`
- `gyak5/launch/gyak5.launch.xml`
- `gyak6/launch/gyak6.launch.xml`
- `gyak10/launch/gyak10.launch.xml`
---

## QoS – Gyors Tipp
- Egyszerű feladatokhoz elég a `depth=10` (reliable default).
- Szükség esetén: `QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)`.

---

## Logolás és Hibakezelés
- `info` normál állapotokra, `warn` gyanús helyzetekre, `error` kivételes hibákra.
- Mindig védd a TF lekérdezéseket és az indexeléseket.

---

## Nevezéktan & Konvenciók
- Topic: `snake_case`, egyértelmű jelentés (`/odom`, `/cmd_vel`, `/scan`, `/path`).
- Frame-ek: `map` (globális), `odom` (lokális drift), `base_link` (robot).
- Namespace több robothoz: `robot1/scan`, `robot2/scan`.

---

## Build & Futás
```bash
# Projekt gyökerében
colcon build --symlink-install
source install/setup.bash

# Node futtatása
ros2 run <package> <entry_point>

# Launch futtatása
ros2 launch <package> <file.launch.xml>
```

---

## Gyakori Segédfüggvények
```python
import math
from tf_transformations import euler_from_quaternion, quaternion_from_euler

def normalize_angle(rad):
    return math.atan2(math.sin(rad), math.cos(rad))
```

---

## Feladat Ellenőrző Lista (Checklista)
- [ ] Csomag és belépési pont rendben (buildelhető, futtatható).
- [ ] Node indítása/lezárása helyes (init, spin, shutdown).
- [ ] Paraméterek deklarálva + kiolvasva.
- [ ] Publisher/Subscriber/Timer létrehozva a feladat szerint.
- [ ] Header és frame id-k következetesek.
- [ ] TF használat, ha koordináta-transzformáció van.
- [ ] RViz Marker(ek), ha vizualizáció kell.
- [ ] Hibakezelés: üres adatok, TF hiánya, indexhatár.
- [ ] Kimenet naplózva (legalább kezdet/összegzés).
- [ ] Launch fájl (ha a feladat ezt kéri).

---

## Rövid Példafelépítés
- Minimális ZH-feladat tipikusan tartalmazza:
  - 1 node osztály (paraméterekkel),
  - 1-2 subscriber (pl. `/odom`, `/scan`) és/vagy 1 publisher,
  - 1 timer (feldolgozási ciklus),
  - opcionálisan TF lekérdezés és Marker publikálás,
  - opcionálisan PointCloud2/Path kezelés,
  - launch fájl és rövid README/komment.

> Tipp: Az ebben a repóban található gyakorlati példák (gyak2–gyak12) mind lefedik a fenti mintákat. Elnevezéseket és paramétereket igazítsd a feladatszöveghez, a struktúra viszont maradjon ezen a sablonon.
