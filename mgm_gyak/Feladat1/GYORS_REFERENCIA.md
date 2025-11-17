# Feladat1 - Gyors referencia (ZH-hoz)

## Feladat követelmények ✓

- [x] „proba1" nevű ROS package
- [x] Feliratkozás `/agent1/odom/ground_truth` (nav_msgs/Odometry)
- [x] Yaw szög számítása quaternion-ból
- [x] Feliratkozás `/path` (nav_msgs/Path)
- [x] Legközelebbi pont keresése
- [x] Legközelebbi pont publikálása `/closest_point` (geometry_msgs/PoseStamped)
- [x] Szögszámítás (legközelebbi pont - yaw)
- [x] Szög publikálása (std_msgs/Float64) `/angle_diff` topicra
- [x] Launch file (node + RViz)
- [x] RViz konfig (LIDAR + legközelebbi pont)

## Gyors build & run

```bash
# 1) Build
cd ~/ros2_ws  # vagy ahol a workspace van
colcon build --packages-select proba1 --symlink-install

# 2) Source
source install/setup.bash

# 3) Launch (node + RViz)
ros2 launch proba1 proba1.launch.xml

# 4) Bag lejátszás (külön terminál)
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 bag play /path/to/bagfile --clock
```

## Kulcsfontosságú kódrészletek

### 1. Quaternion → Yaw

```python
from tf_transformations import euler_from_quaternion

q = [msg.pose.pose.orientation.x,
     msg.pose.pose.orientation.y,
     msg.pose.pose.orientation.z,
     msg.pose.pose.orientation.w]
(roll, pitch, yaw) = euler_from_quaternion(q)
```

### 2. Legközelebbi pont keresése

```python
closest_distance = float('inf')
closest_index = 0

for i, pose_stamped in enumerate(path.poses):
    dx = pose_stamped.pose.position.x - current_x
    dy = pose_stamped.pose.position.y - current_y
    distance = math.sqrt(dx**2 + dy**2)
    
    if distance < closest_distance:
        closest_distance = distance
        closest_index = i

closest_pose = path.poses[closest_index]
```

### 3. Szögszámítás

```python
# Irány a legközelebbi ponthoz (map x tengelyhez képest)
dx = closest_pose.pose.position.x - current_x
dy = closest_pose.pose.position.y - current_y
angle_to_closest = math.atan2(dy, dx)

# Szögkülönbség (célpont irány - saját yaw)
angle_diff = angle_to_closest - current_yaw

# Normalizálás -pi..pi tartományba
angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
```

### 4. Publishers és Subscribers

```python
# Subscriber
self.sub_odom = self.create_subscription(
    Odometry,
    '/agent1/odom/ground_truth',
    self.callback_odom,
    10
)

# Publisher
self.pub_closest = self.create_publisher(
    PoseStamped,
    '/closest_point',
    10
)

# Publikálás
self.pub_closest.publish(closest_pose)
```

## Tesztelés

### Topicok ellenőrzése

```bash
# Listázás
ros2 topic list

# Echo (tartalom)
ros2 topic echo /closest_point
ros2 topic echo /angle_diff

# Info (típus, publisher/subscriber)
ros2 topic info /closest_point
```

### Node ellenőrzése

```bash
ros2 node list
ros2 node info /proba1_node
```

## Gyakori hibák és megoldások

### 1. "use_sim_time" nem működik
- Launch file-ban és RViz-nél is be kell állítani
- Bag-et `--clock` flag-gel kell indítani

### 2. Path üres vagy nincs
- Várj, amíg a bag elküldi az első path üzenetet
- Ellenőrizd: `ros2 topic echo /path`

### 3. RViz nem látszik a LIDAR
- Fixed Frame legyen `map` (vagy megfelelő frame)
- Topic név: `/scan`
- Bag-ben van-e LaserScan adat?

### 4. Import hibák Mac-en
- Normális, nincs ROS 2 telepítve
- Ubuntu-n fog működni

## Package függőségek (package.xml)

```xml
<depend>rclpy</depend>
<depend>nav_msgs</depend>
<depend>geometry_msgs</depend>
<depend>std_msgs</depend>
<depend>tf_transformations</depend>
<depend>tf2_ros</depend>
```

## Launch file sablon (XML)

```xml
<launch>
  <node pkg="proba1" exec="proba1_node" name="proba1_node" output="screen">
    <param name="use_sim_time" value="true"/>
  </node>
  
  <node pkg="rviz2" exec="rviz2" name="rviz2" 
        args="-d $(find-pkg-share proba1)/rviz/proba1.rviz">
    <param name="use_sim_time" value="true"/>
  </node>
</launch>
```

## Hasznos parancsok

```bash
# Workspace tisztítása
rm -rf build/ install/ log/

# Csak egy package build
colcon build --packages-select proba1

# Bag info
ros2 bag info /path/to/bagfile

# TF tree megjelenítése
ros2 run tf2_tools view_frames

# Topic frekvencia
ros2 topic hz /agent1/odom/ground_truth

# Topic bandwidth
ros2 topic bw /scan
```

## RViz beállítások

- **Fixed Frame**: `map`
- **LaserScan** display:
  - Topic: `/scan`
  - Size: 3-5 pixel
  - Style: Flat Squares
- **PoseStamped** display:
  - Topic: `/closest_point`
  - Shape: Arrow
  - Color: Piros/Narancs (jól látható)
- **Path** display:
  - Topic: `/path`
  - Line Style: Lines
  - Color: Zöld
- **Odometry** display:
  - Topic: `/agent1/odom/ground_truth`
  - Shape: Arrow

## Fájlok elhelyezkedése

```
Feladat1/
└── proba1/
    ├── package.xml           ← Függőségek
    ├── setup.py              ← Entry points
    ├── setup.cfg
    ├── README.md             ← Részletes útmutató
    ├── GYORS_REFERENCIA.md   ← Ez a fájl
    ├── resource/
    │   └── proba1
    ├── proba1/
    │   ├── __init__.py
    │   └── proba1_node.py    ← FŐ KÓD
    ├── launch/
    │   └── proba1.launch.xml ← Launch
    └── rviz/
        └── proba1.rviz       ← RViz config
```

## ZH vizsga lépések

1. ✓ Package létrehozása (setup.py, package.xml)
2. ✓ Node implementálása (subscribers, publishers, logika)
3. ✓ Launch file készítése
4. ✓ RViz konfiguráció
5. ✓ Build és tesztelés
6. ✓ Dokumentáció (README)

**Időbecslés:** 30-45 perc tapasztalt fejlesztőnek, 60-90 perc gyakorlónak.

## Kapcsolódó példák a repo-ban

- **gyak3/path.py**: Path építés odometry-ből
- **gyak6/test_scan.py**: LaserScan feldolgozás, legközelebbi pont
- **gyak10/pure_pursuit.py**: Path követés, legközelebbi pont keresés
- **gyak12/node1.py**: Távolság számítás odometry-ből
- **gyak12/node2.py**: TF transform, LaserScan súlypont

Ezek a fájlok mind hasznosak ZH-n, érdemes átolvasni őket!
