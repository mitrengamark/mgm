# Feladat1 - proba1 Package

## Feladat leírása

Készítsen egy „proba1" nevű ROS package-t, amely:

1. **Feliratkozik `/agent1/odom/ground_truth` topicra** (`nav_msgs/Odometry`)
   - Kiszámolja a 'yaw' szöget az orientáció quaternion-ból

2. **Feliratkozik `/path` topicra** (`nav_msgs/Path`)
   - Megkeresi a legközelebbi pontot az útvonalon
   - Publikálja `/closest_point` topicra (`geometry_msgs/PoseStamped`)

3. **Szögszámítás**
   - Kiszámolja a legközelebbi pont és a saját pozíció közötti egyenes szögét (map frame x tengelyéhez képest)
   - Kivonja belőle a yaw szöget
   - Publikálja `std_msgs/Float64` típusú üzenetként `/angle_diff` topicra

4. **Launch file és RViz**
   - Indítja a node-ot és RViz-t
   - RViz-ben látható a LIDAR kép és a legközelebbi pont

## Package struktúra

```
proba1/
├── package.xml              # Package metaadatok és függőségek
├── setup.py                 # Python package setup
├── setup.cfg                # Setup konfiguráció
├── resource/
│   └── proba1              # Marker fájl
├── proba1/
│   ├── __init__.py
│   └── proba1_node.py      # Fő node implementáció
├── launch/
│   └── proba1.launch.xml   # Launch fájl
└── rviz/
    └── proba1.rviz         # RViz konfiguráció
```

## Build és futtatás (Ubuntu 22.04, ROS 2 Humble)

### 1. Környezet beállítása

```bash
# ROS 2 Humble környezet betöltése
source /opt/ros/humble/setup.bash

# Workspace-be navigálás (pl. ~/ros2_ws/src)
cd ~/ros2_ws/src

# Package másolása ide (vagy klónozás)
# cp -r /path/to/Feladat1/proba1 .
```

### 2. Függőségek telepítése

```bash
# Ha még nincsenek telepítve
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  ros-humble-nav-msgs \
  ros-humble-geometry-msgs \
  ros-humble-std-msgs \
  ros-humble-rviz2 \
  python3-tf-transformations
```

### 3. Build

```bash
# Workspace gyökerében
cd ~/ros2_ws
colcon build --packages-select proba1 --symlink-install

# Overlay betöltése
source install/setup.bash
```

### 4. Futtatás

#### Opció A: Launch file-lal (ajánlott)

```bash
# Indítja a proba1_node-ot és RViz-t
ros2 launch proba1 proba1.launch.xml
```

#### Opció B: Külön-külön

```bash
# Terminál 1: Node indítása
ros2 run proba1 proba1_node

# Terminál 2: RViz indítása
rviz2 -d $(ros2 pkg prefix proba1)/share/proba1/rviz/proba1.rviz
```

### 5. Bag file lejátszása (tesztelés)

```bash
# Új terminálban, miután a node fut
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Bag file lejátszása
ros2 bag play /path/to/bagfile --clock
```

**Fontos:** A node és RViz `use_sim_time: true` paraméterrel indul, így szimulált időt használ a bag-ből.

## Ellenőrzés

### Topicok listázása

```bash
ros2 topic list
```

Várható output (részlet):
```
/agent1/odom/ground_truth
/path
/closest_point
/angle_diff
/scan
```

### Topic tartalmának megtekintése

```bash
# Legközelebbi pont
ros2 topic echo /closest_point

# Szögkülönbség
ros2 topic echo /angle_diff
```

### Node információk

```bash
# Node listázása
ros2 node list

# Node részletei
ros2 node info /proba1_node
```

## Kód magyarázat

### proba1_node.py főbb részei

1. **Állapotváltozók**
   - `current_x, current_y`: Saját pozíció (Odometry-ből)
   - `current_yaw`: Orientáció yaw szöge (radiánban)
   - `path`: Útvonal tárolása

2. **Subscribers**
   - `/agent1/odom/ground_truth`: `callback_odom()` hívódik meg
     - Quaternion → Euler konverzió: `euler_from_quaternion()`
     - yaw szög tárolása
   - `/path`: `callback_path()` hívódik meg
     - Path tárolása

3. **Publishers**
   - `/closest_point`: Legközelebbi pont (PoseStamped)
   - `/angle_diff`: Szögkülönbség (Float64)

4. **process_path() metódus**
   - Legközelebbi pont keresése: euklideszi távolság minimalizálása
   - Szögszámítás: `atan2(dy, dx) - current_yaw`
   - Normalizálás -π..π tartományba

### RViz konfiguráció

- **Fixed Frame**: `map`
- **Megjelenített elemek**:
  - Grid: Koordináta rács
  - LaserScan: LIDAR pontfelhő (`/scan`)
  - ClosestPoint: Legközelebbi pont (`/closest_point`)
  - Path: Útvonal (`/path`)
  - Odometry: Robot pozíció és orientáció (`/agent1/odom/ground_truth`)

## Hibaelhárítás

### "Import could not be resolved" hibák

Ez normális Mac-en, mert nincs telepítve ROS 2. Ubuntu-n nem jelentkezik.

### "No module named 'tf_transformations'"

```bash
sudo apt install python3-tf-transformations
```

### Launch file nem találja az RViz konfigot

Bizonyosodj meg róla, hogy a build után újra forrásoltad:
```bash
source install/setup.bash
```

### Bag nem játszik le

Ellenőrizd:
- `use_sim_time: true` paraméter be van-e állítva
- `--clock` flag használva van-e a bag play-nél

## ZH tippek

1. **Quaternion → Euler konverzió**
   ```python
   from tf_transformations import euler_from_quaternion
   q = [qx, qy, qz, qw]
   (roll, pitch, yaw) = euler_from_quaternion(q)
   ```

2. **Legközelebbi pont keresése**
   ```python
   min_dist = float('inf')
   for point in path.poses:
       dist = math.sqrt((point.x - current_x)**2 + (point.y - current_y)**2)
       if dist < min_dist:
           min_dist = dist
           closest = point
   ```

3. **Szögszámítás (atan2)**
   ```python
   angle = math.atan2(dy, dx)  # -pi..pi tartományban
   ```

4. **Szög normalizálás**
   ```python
   angle = math.atan2(math.sin(angle), math.cos(angle))
   ```

## További információk

- ROS 2 dokumentáció: https://docs.ros.org/en/humble/
- tf_transformations: http://wiki.ros.org/tf2/Tutorials
- nav_msgs: http://docs.ros.org/en/api/nav_msgs/html/index-msg.html
