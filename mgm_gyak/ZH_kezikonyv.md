# ZH készültető – ROS2 (rclpy) gyors kézikönyv

Cél: a ZH-n tipikus ROS2 feladatokat gyorsan megoldani a meglévő példák mintájára. Célkörnyezet: Ubuntu 22.04 (ROS 2 Humble). Mac-en nincs ROS2 – ott csak olvasd/gyakorold a mintákat.

## Ubuntu 22.04 (Humble) gyors beállítás

```zsh
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  ros-humble-rviz2 \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  ros-humble-sensor-msgs-py \
  python3-tf-transformations

# ROS 2 környezet betöltése (Humble)
source /opt/ros/humble/setup.bash
```

## Build és futtatás (colcon)

```zsh
# Munkahelyi könyvtár (workspace) gyökerében
colcon build --symlink-install

# Új shellben forrásold az overlay-t (Ubuntu-n jellemzően bash)
source install/setup.bash  # zsh esetén: source install/setup.zsh

# Egy csomag futtatása
ros2 run gyak2 talker      # példában a publisher entry pointja: talker

# Launch (XML)
ros2 launch gyak10 gyak10.launch.xml

# Hasznos: témák, nódfák, echo
ros2 topic list
ros2 node list
ros2 topic echo /odom
```

Megjegyzés: Új terminálnál előbb a ROS 2 base környezetet is töltsd be:

```zsh
source /opt/ros/humble/setup.bash
```

Tipp: Ha bag-et játszol vissza: `ros2 bag play <bag_mappa>` és `use_sim_time: true` paraméterekkel dolgozz.

---

## 1) Publisher/Subscriber minta (gyak2)

- Minta publisher: `gyak2/gyak2/publisher.py` – alap üzenetküldés időzítővel.
- Feladat a ZH-n gyakran: írj subscriber-t egy adott topicra, és logolj / dolgozz az adatokkal.

Példa, erősen kommentelt Subscriber váz (String-re):

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Subscriber_Node(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        # 1) Feliratkozás: típus, topic, callback, queue depth (QoS)
        self.subscription = self.create_subscription(
            String,          # üzenet típusa
            'chatter',       # topic neve – egyezzen a publisherrel
            self.cb_msg,     # visszahívás
            10               # QoS queue
        )
        # prevent unused variable warning
        self.subscription

    def cb_msg(self, msg: String):
        # 2) Üzenet feldolgozása
        self.get_logger().info(f'Received: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = Subscriber_Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Kulcs API-k: `create_publisher`, `create_subscription`, `create_timer`, `get_logger()`.

---

## 2) Odom → Path építés (gyak3)

- Fájl: `gyak3/gyak3/path.py`. Odometry üzenetekből `nav_msgs/Path`-ot épít és publikál.
- Lényeges pontok:
  - Paraméterek: `path_topic_name`, `odom_topic_name`, `max_size`.
  - `PoseStamped`-et append-el a `Path.poses`-hez, majd publikál.
  - Ha túl hosszú, régi elemek törlése (gyorsítás, memória).

Tipikus ZH-kérés: „Építs path-ot /odom-ból és publikáld /path-ra.” – Ezt a fájlt lehet mintázni 1:1-ben.

---

## 3) Paraméterezés és YAML (gyak4)

- YAML példa: `gyak4/config/path_param.yaml`

```yaml
/**:
  ros__parameters:
    path_topic_name: /path
    max_size: 500
    use_sim_time: true
```

- Használat: launch-ból betölthető, vagy node-ban `declare_parameter` + `get_parameter`.
- ZH-n: kérhetik, hogy paraméterrel állítható legyen a topic neve, max hossz stb.

---

## 4) RViz Marker(ek) publikálása (gyak5)

- Fájl: `gyak5/gyak5/test_viz.py`
- Mutatja, hogyan:
  - `MarkerArray`-t építs, különböző típusokkal (`CUBE`, `SPHERE`, `SPHERE_LIST`).
  - Élettartam (`lifetime`), skálák, színek.
  - Egyszerű pálya- és téglalap-becslés publikálása (min/max x,y alapján bounding box).

ZH tipp: ha vizualizáció kell, válassz egyszerű Marker típust (pl. SPHERE), és állítsd a header-t és scale-t korrekten.

---

## 5) LaserScan feldolgozás, PointCloud, legközelebbi pont (gyak6)

- Fájl: `gyak6/gyak6/test_scan.py`
- Lépések:
  - `LaserScan.ranges` bejárása; min/max tartomány szűrése.
  - Polár → derékszögű: `x = r*cos(θ)`, `y = r*sin(θ)`.
  - MarkerArray és `sensor_msgs_py.point_cloud2.create_cloud_xyz32`-vel `PointCloud2` publikálása.
  - Legközelebbi pont meghatározása és `PoseStamped` publikálása.

ZH tipp: számításoknál figyelj a header-ekre és `Duration` konverzióra.

---

## 6) Egyszerű Ackermann robot + Pure Pursuit (gyak10)

- Kódok: `gyak10/gyak10/ack_robot.py` és `gyak10/gyak10/pure_pursuit.py`.
- `ack_robot.py`:
  - `Twist` parancs (`/cmd_vel`) alapján integrálja a mozgást, publikál `/odom`-ot, TF-et (`odom`→`base_link`).
  - Paraméter: `wheel_base`.
- `pure_pursuit.py`:
  - Paraméterek: `wheelbase`, `lookahead_distance`, `max_speed`, `max_steering_speed`.
  - Beolvassa a Path-ot (`/path`), TF-ből becsli a robot pozíciót (`map`↔`base_link`).
  - Kiszámítja a legközelebbi és a lookahead pontot, majd tisztított szögkülönbséggel kormányzási parancsot ad ki (`Twist`).

Tipikus integráció parancsok:

```zsh
# 1) Robot szimulátor
ros2 run gyak10 ack_robot

# 2) Útvonal publikáló (pl. gyak11-ből, lásd lent), vagy saját path node
ros2 run gyak11 pub_path

# 3) Irányító
ros2 run gyak10 pure_pursuit
```

Figyelem: a TF elérhetőségét mindig ellenőrizd (`can_transform`).

---

## 7) Útvonal fájlból publikálása és mentése (gyak11)

- Publikálás fájlból: `gyak11/gyak11/pub_path.py`
  - Paraméter: `text_file_name` (alapértelmezett a repo-ban lévő `param/positions.txt`).
  - Soronként: x, y, z, yaw_deg értékek; kvaternió konverzió `quaternion_from_euler`-rel.
  - `MarkerArray`-jal vizuális sarokpontok (min–max) jelölése.

- Mentés fájlba: `gyak11/gyak11/save_path.py`
  - Odom-ból távolságküszöb szerint (param: `distance`) feljegyzi a pontokat.
  - Ha a frame nem `map`, TF-fel `map`-ba transzformál, utána írja a sort: `x, y, z, yaw_deg`.

ZH tipp: fájl megnyitás/bezárás, és TF fallback kezelése.

---

## 8) Mintafeladatok a ZH-ra (gyak12)

- `node1.py`: összegzi a megtett távolságot Odom alapján; ha egész métert lép, újrapublikálja az aktuális Odom-ot `/poz`-ra és folyamatosan publikálja az össztávot `/elmozdulas`-on (`Float64`).
- `node2.py`: `LaserScan`-ból pontfelhőt és súlypontot számol; Marker és `PoseStamped` publikálása; TF-fel `map`-ba transzformált pozíció publikálása.

Sablon-lépések ehhez:
- Feliratkozás (`create_subscription`) → feldolgozás → publikálás (`create_publisher`).
- Header-ek helyes beállítása, QoS sorhossz, `Duration`.
- TF ellenőrzés: `can_transform(...)` + `lookup_transform(...)`.

---

## 9) Launch file sablonok

XML sablon (ROS2):

```xml
<launch>
  <node pkg="gyak2" exec="publisher" name="publisher_node" output="screen">
    <param name="use_sim_time" value="true"/>
  </node>
</launch>
```

Python sablon (ha engedélyezett):

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gyak2',
            executable='publisher',
            name='publisher_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
    ])
```

---

## 10) Gyors API emlékeztető

- Node létrehozás: `class X(Node): super().__init__('name')`
- Publisher: `self.create_publisher(MsgType, '/topic', 10)`
- Subscriber: `self.create_subscription(MsgType, '/topic', cb, 10)`
- Timer: `self.create_timer(0.1, cb)`  # 10 Hz
- Logger: `self.get_logger().info('...')`
- Paraméter: `self.declare_parameter('p', default); self.get_parameter('p').value`
- TF ellenőrzés: `buffer.can_transform('to','from', time, timeout)`
- Kvaternió ↔ Euler: `tf_transformations` modul
- PointCloud2: `sensor_msgs_py.point_cloud2.create_cloud_xyz32(header, points)`

---

## 11) Minimális vizsga-lépéssor

1) Készítsd el a node-ot a fenti mintákból (publisher/subscriber/scan/path/tf).
2) Add hozzá a `setup.py` entry pointját (ha új executable kell) és `package.xml`-ben a függőségeket (std_msgs/nav_msgs/geometry_msgs/visualization_msgs/tf2_ros stb.).
3) `colcon build --symlink-install` → `source install/setup.bash` (zsh esetén `setup.zsh`).
4) Futtasd `ros2 run ...` vagy `ros2 launch ...` paranccsal.
5) Ellenőrzés: `ros2 topic list/echo`, RViz marker/Path (ha van környezet).

Ezekkel a mintákkal a repo-ban lévő feladatok 90%+ lefedhető. Ha akarsz, adjak külön, kommentelt sablonfájlokat is a `templates/` mappába?
