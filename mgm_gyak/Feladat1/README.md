# Feladat1 - proba1 Package

## 📋 Feladat teljesítve ✓

ROS 2 package, amely:
- ✅ Feliratkozik `/agent1/odom/ground_truth`-ra (nav_msgs/Odometry)
- ✅ Kiszámolja a yaw szöget quaternion-ból
- ✅ Feliratkozik `/path`-ra (nav_msgs/Path)
- ✅ Megkeresi a legközelebbi pontot az útvonalon
- ✅ Publikálja `/closest_point`-ra (geometry_msgs/PoseStamped)
- ✅ Kiszámolja és publikálja a szögkülönbséget (std_msgs/Float64)
- ✅ Launch file (node + RViz)
- ✅ RViz konfig (LIDAR + legközelebbi pont megjelenítés)

## 🚀 Gyors indítás (Ubuntu 22.04, ROS 2 Humble)

```bash
# 1. Build
cd ~/ros2_ws
colcon build --packages-select proba1 --symlink-install
source install/setup.bash

# 2. Launch (node + RViz)
ros2 launch proba1 proba1.launch.xml

# 3. Bag lejátszás (külön terminál)
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 bag play /path/to/bagfile --clock
```

## 📁 Fájlok

- **proba1/** - ROS 2 package
  - `proba1_node.py` - Fő node implementáció (részletesen kommentezve)
  - `proba1.launch.xml` - Launch file
  - `proba1.rviz` - RViz konfiguráció
  - `package.xml`, `setup.py` - Package metaadatok

- **README.md** - Részletes dokumentáció
- **GYORS_REFERENCIA.md** - ZH-s gyors segédlet
- **MEGOLDAS_LEPESROL_LEPESRE.md** - Teljes, lépésenkénti magyarázat
- **Feladat1.txt** - Eredeti feladat specifikáció

## 📚 Dokumentumok

1. **README.md**
   - Package struktúra
   - Build & futtatás részletesen
   - Ellenőrzési parancsok
   - Hibaelhárítás
   - ZH tippek

2. **GYORS_REFERENCIA.md**
   - Követelmények checklist
   - Kulcsfontosságú kódrészletek
   - Build & run parancsok
   - Gyakori hibák
   - Hasznos parancsok

3. **MEGOLDAS_LEPESROL_LEPESRE.md**
   - 8 lépéses teljes magyarázat
   - Minden kódsor részletezése
   - Geometria, matematika háttér
   - ZH stratégia és időbeosztás
   - Kapcsolódó témák

## 🎯 Kulcs koncepciók

### 1. Quaternion → Euler (yaw)
```python
from tf_transformations import euler_from_quaternion
q = [qx, qy, qz, qw]
(roll, pitch, yaw) = euler_from_quaternion(q)
```

### 2. Legközelebbi pont keresése
```python
min_dist = float('inf')
for point in path.poses:
    dist = sqrt((point.x - current_x)**2 + (point.y - current_y)**2)
    if dist < min_dist:
        min_dist = dist
        closest = point
```

### 3. Szögszámítás
```python
angle_to_target = atan2(dy, dx)
angle_diff = angle_to_target - current_yaw
# Normalizálás -pi..pi
angle_diff = atan2(sin(angle_diff), cos(angle_diff))
```

## 🔍 Ellenőrzés

```bash
# Topicok
ros2 topic list
ros2 topic echo /closest_point
ros2 topic echo /angle_diff

# Node info
ros2 node info /proba1_node
```

## 💡 ZH tippek

- **Package struktúra:** setup.py, package.xml, __init__.py
- **Függőségek:** rclpy, nav_msgs, geometry_msgs, std_msgs, tf_transformations
- **Topic nevek:** Pontosan a feladat szerint!
- **use_sim_time:** MINDIG true bag lejátszásnál
- **--clock flag:** Bag play-nél kötelező
- **Tesztelés:** `ros2 topic echo` gyakran használd
- **Kommentek:** Segítenek a ZH-n, ha elakadsz

## 📖 Kapcsolódó példák a repo-ban

- `gyak3/path.py` - Path építés, paraméterezés
- `gyak6/test_scan.py` - LaserScan, legközelebbi pont
- `gyak10/pure_pursuit.py` - Path követés, lookahead
- `gyak12/node1.py` - Odometry, távolság számítás
- `gyak12/node2.py` - TF transform, LaserScan súlypont

Ezeket érdemes átnézni a ZH előtt!

## ⚙️ Technikai részletek

- **Nyelv:** Python 3
- **ROS:** ROS 2 Humble
- **OS:** Ubuntu 22.04
- **Üzenet típusok:** nav_msgs/Odometry, nav_msgs/Path, geometry_msgs/PoseStamped, std_msgs/Float64
- **Függőségek:** rclpy, tf_transformations, math

## 🎓 Tanulási célok

✅ ROS 2 Python package létrehozása  
✅ Subscriber és Publisher használat  
✅ Üzenet típusok (Odometry, Path, PoseStamped, Float64)  
✅ Quaternion → Euler konverzió  
✅ Geometriai számítások (távolság, szög)  
✅ Launch file XML  
✅ RViz konfiguráció  
✅ Bag file használat  

---

**Ha kérdésed van, nézd meg a MEGOLDAS_LEPESROL_LEPESRE.md fájlt - minden részletesen le van írva!**
