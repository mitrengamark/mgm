# Gyak9 - Többrobotikus LaserScan Aggregáció TF2 Transzformációval

## Célkitűzés

Ez a gyakorlat a **ROS2 TF2 transzformációs rendszer** gyakorlati alkalmazásait mutatja be több robot segítségével. A feladat az, hogy a különböző robotok által érzékelt LaserScan adatokat **egyetlen világkoordinátarendszerbe** (map frame) transzformáljunk és egy közös PointCloud2-ben egyesítsük.

### Mit tanulsz meg:
- TF2 Buffer és TransformListener használata
- StaticTransformBroadcaster a kinetikus lánc kiegészítésére
- LaserScan → PointCloud2 konverzió
- Pont felhő transzformáció (`tf2_sensor_msgs.do_transform_cloud`)
- Több robot koordinációja ROS2-ben
- Gazebo szimulációban történő munkavégzés namespaceokkal

---

## A projekt szerkezete

```
gyak9/
├── gyak9/
│   ├── __init__.py
│   └── scan_tf.py           # A fő csomópont
├── launch/
│   ├── gyak9.launch.xml     # Launch fájl a szimuláció elindításához
│   └── ns_robot.launch.py   # Namespaced robot spawn
├── rviz/
│   └── gyak9.rviz          # RViz konfiguráció
└── test/                    # Tesztfájlok
```

---

## Fő komponens: `scan_tf.py`

### Mi az a ScanHandler?

A `ScanHandler` osztály a csomópont szíve. Három fő dolgot csinál:

#### 1. **LaserScan előfizetés (Subscription)**
```python
self.sub_scan_robot1 = self.create_subscription(LaserScan, '/robot1/scan', ...)
self.sub_scan_robot2 = self.create_subscription(LaserScan', '/robot2/scan', ...)
```
- Befizetődik az első robot (`/robot1/scan`) és a második robot (`/robot2/scan`) lézer szenzor adatira
- Mindkét robotnak saját, namespaced topicja van

#### 2. **LaserScan → PointCloud2 konverzió**
Az `laserscan_to_pointcloud` metódus:
- Fogadja a poláris koordinátás LaserScan adatot (szög, távolság)
- Konvertálja Descartes-koordinátákra (X, Y, Z)
- Létrehoz egy PointCloud2 üzenetet ugyanabban a frame-ben, mint a LaserScan

```
PointCloud2 felépítése:
- header.frame_id: pl. "robot1/base_scan" (ahonnan jött az adat)
- points[]: X, Y, Z koordináták az adott robot kerete szerint
```

#### 3. **TF2 transzformáció és pont felhő egyesítés**

A `timer_callback` minden 1 másodpercben:

1. **Ellenőrzi**, hogy létezik-e transzformáció `map → robot1/base_scan` és `map → robot2/base_scan` között
   ```python
   if self.tf_buffer.can_transform('map', 'robot1/base_scan', time, timeout=rclpy.duration.Duration(seconds=0.5)):
   ```

2. **Lekérdezi** a legfrissebb transzformációt
   ```python
   transform = self.tf_buffer.lookup_transform('map', frame_id, time)
   ```

3. **Transzformálja** az egyes pont felhőket a `map` keretbe
   ```python
   cloud_transformed = tf2_sensor_msgs.do_transform_cloud(cloud, transform)
   ```

4. **Egyesíti** az összes transzformált pont felhőt
   ```python
   merged_cloud = self.merge_pointclouds([cloud1_map, cloud2_map])
   ```

5. **Publikálja** az egyesített pont felhőt az `/cloud` topicon

---

## TF2 és koordinátarendszerek

### Miért kell TF2 transzformáció?

Képzeld el, hogy két robot van a szobában:
- **Robot1** az origótól 2 méterre balra van → saját `robot1/odom` kerete
- **Robot2** az origótól 2 méterre jobbra van → saját `robot2/odom` kerete
- A **map** az abszolút világkoordináta-rendszer

Mindegyik robot saját "szemüvegből" látja a világot. Ha közvetlenül összevonnánk a LaserScan adatokat, összekeveredne a koordinátarendszer. A TF2 ezt megoldja.

### A kinetikus lánc (Kinematic Chain):

```
map
├── static_transform → robot1/odom (pozíció: -2, 0, 0)
│   └── tf (from robot1) → robot1/base_footprint
│       └── tf (from robot1) → robot1/base_scan
│
└── static_transform → robot2/odom (pozíció: 2, 0, 0)
    └── tf (from robot2) → robot2/base_footprint
        └── tf (from robot2) → robot2/base_scan
```

A `do_transform_cloud` ezeket az átváltásokat használja, hogy a pont felhő koordinátáit a `map` keretbe transzponálja.

---

## Launch fájl (`gyak9.launch.xml`)

### Mit csinal?

```xml
<launch>
  <!-- 1. Gazebo szerver és kliens indítása -->
  <node pkg="gazebo_ros" exec="gzserver" args="..." />
  <node pkg="gazebo_ros" exec="gzclient" args="..." />

  <!-- 2. Robot spawn namespaceokkal -->
  <include file="$(find-pkg-share gyak9)/launch/ns_robot.launch.py">
    <arg name="robot_name" value="robot1" />
    <arg name="x_pose" value="-2.0" />
    <arg name="y_pose" value="0.0" />
  </include>
  
  <include file="...">
    <arg name="robot_name" value="robot2" />
    <arg name="x_pose" value="2.0" />
    <arg name="y_pose" value="0.0" />
  </include>

  <!-- 3. Statikus transzformációk (map → robot/odom) -->
  <node pkg="tf2_ros" exec="static_transform_publisher"
        args="0 0 0 0 0 0 1 map robot1/odom" />
  <node pkg="tf2_ros" exec="static_transform_publisher"
        args="0 0 0 0 0 0 1 map robot2/odom" />

  <!-- 4. A main csomópont (scan_tf.py) indítása -->
  <node pkg="gyak9" exec="test_scan"
        args="--ros-args -p map_frame:=map -p use_sim_time:=true" />

  <!-- 5. RViz indítása -->
  <node pkg="rviz2" exec="rviz2" args="-d $(find-pkg-share gyak9)/rviz/gyak9.rviz" />
</launch>
```

**Fontos részletek:**

- **`use_sim_time:=true`**: Gazebo szimulációs idő használata (nem valós idő)
- **`map_frame:=map`**: A cél koordinátarendszer neve
- **Statikus transzformációk**: Az `args` paraméter: `x y z qx qy qz qw parent child`

---

## Robot spawn (`ns_robot.launch.py`)

### Namespace használat

A namespacing lehetővé teszi, hogy ugyanaz az alkalmazás több példánya fusson egymástól függetlenül:

```python
def generate_launch_description():
    robot_name = LaunchConfiguration('robot_name')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    
    # A robot frame-eit átnevezik:
    # base_scan → robot1/base_scan (robot1 esetén)
    # base_scan → robot2/base_scan (robot2 esetén)
```

Ez azt jelenti, hogy mindegyik robotnak saját topicjai vannak:
- `/robot1/scan` (nem `/scan`)
- `/robot2/scan` (nem `/scan`)
- `/robot1/tf` (nem `/tf`)
- `/robot2/tf` (nem `/tf`)

---

## RViz konfiguráció (`gyak9.rviz`)

### Megjelenítés

Az RViz beállítva van arra, hogy:

1. **Merged PointCloud2** (`/cloud`)
   - Az összes robot által érzékelt pont egy közös keretben
   - A `map` kerethez rögzített

2. **Markers** (`/viz`)
   - Vizuális jelölők a csomópont által publikálva

3. **Robot1 csoport:**
   - RobotModel: a robot 3D modellje
   - LaserScan: az érzékelt pontok az eredeti `robot1/base_scan` keretben
   - Topic: `/robot1/scan`

4. **Robot2 csoport:**
   - RobotModel: a robot 3D modellje
   - LaserScan: az érzékelt pontok az eredeti `robot2/base_scan` keretben
   - Topic: `/robot2/scan`

**Fixed Frame**: `robot1/odom` (vagy `map` - az élő konfigurációban módosítható)

---

## Futtatás és működés

### Előfeltételek

```bash
# TurtleBot3 szimuláció szükséges
sudo apt-get install ros-humble-turtlebot3-gazebo
sudo apt-get install ros-humble-tf2-sensor-msgs

# Workspace építése
cd ~/codes/mgm/mgm
colcon build --packages-select gyak9
```

### Indítás

```bash
# Beállítás (egy terminálban egyszer)
export TURTLEBOT3_MODEL=waffle
source install/setup.bash

# Szimuláció indítása
ros2 launch gyak9 gyak9.launch.xml
```

Ez a parancs egyszerre indít:
1. Gazebo szimulátort (2 robot, 6-7 másodperc)
2. RViz-et (a pont felhő és robotok megjelenítéséhez)
3. A `scan_tf` csomópontot

### Ellenőrzés

Külön terminálban:

```bash
# Topicok listája
ros2 topic list

# Az egyesített pont felhő topicjának ellenőrzése
ros2 topic echo /cloud | head -20

# TF fa megjelenítése
ros2 run tf2_tools view_frames.py
```

**Várható kimenet:**
- `/cloud`: PointCloud2, amely mindkét robot LaserScan-jét tartalmazza
- `/robot1/scan` és `/robot2/scan`: Az eredeti LaserScan-ek
- Több TF frame: `map`, `robot1/odom`, `robot1/base_footprint`, `robot1/base_scan`, stb.

---

## Főbb tanulságok

### 1. **TF2 transzformáció**
- Soha ne hardkódolj koordinátákat; használd a TF2-t!
- A `can_transform` időtúllépés fontos (0.5s) a bolygó működési zavarok kezeléséhez

### 2. **Pont felhő transzformáció**
- `tf2_sensor_msgs.do_transform_cloud` az aranystandardjava
- Ne felejtsd el a frame_id-t és az időbélyeget (timestamp)

### 3. **Namespace-ek**
- Használj namespace-eket több robot koordinációjához
- Ez megakadályozza az ütközéseket a topicok és frame-ek között

### 4. **Statikus transzformációk**
- A `map → robot/odom` kapcsolat általában statikus (nem változik)
- A robot által publikált TF-ek (`robot/base_footprint → robot/base_scan`) dinamikusak

### 5. **ROS2 szimulációban**
- Használd `use_sim_time:=true` szinkronizációhoz
- A Gazebo adja az időt, nem a valós rendszeróra

---

## Gyakori hibák és megoldások

### "Cannot transform between frames"
- Ellenőrizd, hogy a kinetikus lánc teljes-e
- Használj `ros2 run tf2_tools view_frames.py` diagnosztikához

### Pont felhő üres vagy rosszul pozicionált
- Ellenőrizd, hogy `can_transform` igaz-e
- Nézd meg az időbélyegeket (nem szinkronizált?)

### Gazebo nem indul
- Ellenőrizd: `export TURTLEBOT3_MODEL=waffle`
- Szükséges: `ros-humble-gazebo-ros-pkgs` és `ros-humble-turtlebot3-gazebo`

---

## Továbbfejlesztési ötletek

1. **Dinamikus robot szám**: Tetszőleges számú robot támogatása
2. **Pont felhő szűrés**: Csak közeli pontok megtartása (távolsági küszöb)
3. **Intenzitás mérték**: Különböző színek az egyes robotok szerint
4. **Valós robotok**: Gazebo helyett valós TurtleBot3 robotok
5. **Perfomancia**: ROS2-filter node-ok vagy MoveIt! integráció

---

## Hasznos parancsok

```bash
# RViz konfigurációs fájl megnyitása
gedit ~/codes/mgm/mgm/mgm_gyak/gyak9/rviz/gyak9.rviz

# LaserScan adatok vizsgálata
ros2 topic echo /robot1/scan --once | head -30

# Pont felhő méret ellenőrzése
ros2 topic echo /cloud --once | grep "width\|height"

# Csomópont logja (rclcpp DEBUG szintű)
ros2 run gyak9 test_scan --ros-args --log-level debug
```

---

## Referenciák

- [ROS2 TF2 Dokumentáció](https://docs.ros.org/en/humble/Concepts/Intermediate/Tf2/Tf2.html)
- [sensor_msgs/PointCloud2](https://docs.ros2.org/latest/api/sensor_msgs/msg/PointCloud2.html)
- [TurtleBot3 Gazebo](https://docs.turtlebot.org/en/humble/simulation/gazebo/overview.html)
- [Launch XML Formátum](https://docs.ros.org/en/humble/Concepts/Basic/About-Launch.html)

---

**Készült**: 2025. december
**ROS verzió**: ROS 2 Humble
**Tesztelve**: TurtleBot3 Waffle Gazebo szimulációban
