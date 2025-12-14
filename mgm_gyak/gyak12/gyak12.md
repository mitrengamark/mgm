# Gyak12 - ROS2 Zhk Vizsga Gyakorló Feladat

## Célkitűzés

Ez a gyakorlat a **ROS2 vizsga gyakorló feladatainak** megoldása. Két csomópont készítésével gyakoroljuk:

1. **Node1 (`node1.py`)**: Odometria feldolgozása, távolság számolása, feltételes publikálás
2. **Node2 (`node2.py`)**: LaserScan feldolgozása, pont felhő készítés, súlyponti pont számítás, TF transzformáció

### Mit tanulsz meg:
- Odometria adatok feldolgozása és távolság számolása
- Feltételes publikálás logika
- LaserScan adatok feldolgozása (poláris → Descartes)
- PointCloud2 üzenet létrehozása
- Matematikai műveletek pont felhőkön (súlyponti pont = centroid)
- TF2 transzformáció (lokális → globális koordináta-rendszer)
- Vizualizációs markerek a RViz-ben

---

## A projekt szerkezete

```
gyak12/
├── feladat.txt               # Vizsga feladat leírása
├── gyak12/
│   ├── __init__.py
│   ├── node1.py             # Odometria feldolgozás
│   └── node2.py             # LaserScan feldolgozás
├── launch/
│   └── gyak12.launch.xml    # Launch fájl (mindkét node-ot indítja)
├── rviz/
│   └── gyak12.rviz         # RViz konfiguráció
└── test/                    # Tesztfájlok
```

---

## Komponens 1: Node1 - Odometria Feldolgozása (`node1.py`)

### Célja

A `node1.py` egy **ZhPub** csomópont, amely:
1. Feliratkozik az `/odom` topicra (robot odometriája)
2. Kiszámolja a robot **elmozdulásait** (távolságot két üzenet között)
3. **Összegzi az összes elmozdulást** és publikálja az `/elmozdulas` topicra (Float64)
4. **Feltételesen publikálja** az odometriát az `/poz` topicra (csak ha az egész méterek száma növekszik)

### Szálkód magyarázata

#### 1. Inicializálás

```python
class ZhPub(Node):
    def __init__(self):
        super().__init__('zh_node')

        # Subscriber: odometria fogadása
        self.subscription = self.create_subscription(Odometry, '/odom', self.callback_odom, 1)
        
        # Publisher: odometria (feltételes)
        self.publisher_odom = self.create_publisher(Odometry, '/poz', 1)
        
        # Publisher: teljes távolság
        self.publisher_dist = self.create_publisher(Float64, '/elmozdulas', 1)

        # Tárolók
        self.dist_msg = Float64()  # Teljes elmozdulás (0.0 kezdeti érték)
        self.last_msg = Point()     # Utolsó feldolgozott pozíció
```

**Publikálásított topicok:**
- `/elmozdulas` (Float64): a robot összes bejárt távolsága
- `/poz` (Odometry): az odometria üzenet, de csak ha 1 teljes métert haladt

**Feliratkozott topic:**
- `/odom` (Odometry): a robot odometriája

#### 2. Odometria callback: távolság számolása

```python
def callback_odom(self, msg: Odometry):
    # Euclidean távolság az utolsó pozíciótól
    elmozdulas = math.sqrt(
        (msg.pose.pose.position.x - self.last_msg.x)**2 + 
        (msg.pose.pose.position.y - self.last_msg.y)**2
    )
```

Ez a **Pitagorasz-tétel** használatával kiszámolja a távolságot az előző pozícióból az aktuálisba:

$$d = \sqrt{(x_{új} - x_{régi})^2 + (y_{új} - y_{régi})^2}$$

A Z koordinátát figyelmen kívül hagyjuk (síkmozgás feltételezése).

#### 3. Feltételes publikálás

```python
if math.floor(self.dist_msg.data) < math.floor(self.dist_msg.data + elmozdulas):
    self.publisher_odom.publish(msg)
```

Ez **csak akkor** publikálja az odometriát, ha az egész méterek száma nő:

- Jelenlegi teljes távolság: 1.9 m → `floor(1.9) = 1`
- Új elmozdulás: 0.3 m
- Új teljes távolság: 2.2 m → `floor(2.2) = 2`
- **2 > 1** → publikálunk!

Például:
```
Üzenet 1: távolság = 0.5m → 0.5m teljes → nem publikálunk (1 < 1 hamis)
Üzenet 2: távolság = 0.6m → 1.1m teljes → PUBLIKÁLUNK! (1 < 1 hamis, de 0 < 1 igaz)
Üzenet 3: távolság = 0.2m → 1.3m teljes → nem publikálunk (1 < 1 hamis)
Üzenet 4: távolság = 0.8m → 2.1m teljes → PUBLIKÁLUNK! (2 > 1 igaz)
```

#### 4. Teljes távolság frissítése

```python
self.dist_msg.data += elmozdulas
self.publisher_dist.publish(self.dist_msg)

# Utolsó pozíció frissítése
self.last_msg = msg.pose.pose.position
```

A teljes elmozdulás folyamatosan nő, és minden alkalommal publikálódik.

---

## Komponens 2: Node2 - LaserScan Feldolgozása (`node2.py`)

### Célja

A `node2.py` egy **ZhPub2** csomópont, amely:
1. Feliratkozik a `/scan` topicra (lézer szenzor adatai)
2. A **poláris koordinátákat Descartes-koordinátákra** alakítja
3. **PointCloud2 üzenetet** hoz létre és publikálja az `/cloud` topicra
4. Kiszámolja a pont felhő **súlyponti pontját** (centroid)
5. A súlyponti pontot **TF transzformációval** a `map` keretbe transzformálja
6. Publikálja a **Marker** (zöld gömb) és **PoseStamped** üzeneteket

### Szálkód magyarázata

#### 1. Inicializálás

```python
class ZhPub2(Node):
    def __init__(self):
        super().__init__('zh_node2')

        # TF Buffer és Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publisherek
        self.pub_points = self.create_publisher(PointCloud2, "/cloud", 10)
        self.pub_marker = self.create_publisher(Marker, "/marker", 10)
        self.pub_pose = self.create_publisher(PoseStamped, "/pose", 10)
        
        # Subscriber
        self.subscription = self.create_subscription(LaserScan, "/scan", self.scan_callback, 1)
```

**Publikálásított topicok:**
- `/cloud` (PointCloud2): a lézer pontok Descartes-koordinátákban
- `/marker` (Marker): zöld gömb a súlyponti pontban
- `/pose` (PoseStamped): súlyponti pont a `map` keretben

#### 2. LaserScan feldolgozása

```python
def scan_callback(self, msg: LaserScan):
    newPoint = []
    x = y = z = 0.0
    valid_points = 0
    
    # Végigmegyünk a LeserScan méréseken
    for i in range(0, len(msg.ranges)):
        angle = msg.angle_min + i * msg.angle_increment
        
        # Szűrés: csak valid tartományban lévő mérések
        if (msg.ranges[i] < msg.range_min or msg.ranges[i] > msg.range_max):
            continue
```

A LaserScan egy **poláris koordinátarendszerbeli** mérés: szög és távolság.

- `msg.angle_min`: a lézer kezdő szöge (pl. -π rad)
- `msg.angle_increment`: szögközök közötti távolság (pl. π/180 rad)
- `msg.ranges[i]`: a távolság az `i`-edik szög alatt

#### 3. Poláris → Descartes transzformáció

```python
# Poláris → Descartes
x_temp = msg.ranges[i] * math.cos(angle)
y_temp = msg.ranges[i] * math.sin(angle)
z_temp = msg.ranges[i]  # SPECIÁLIS: z = távolság!
newPoint.append([x_temp, y_temp, z_temp])
```

A szokásos konverzió:

$$x = r \cdot \cos(\theta)$$
$$y = r \cdot \sin(\theta)$$

**Fontos:** A feladatban z helyett a **távolság értékét** tesszük be!

```
Poláris (angle=45°, r=1m) → Descartes:
x = 1 * cos(45°) = 0.707
y = 1 * sin(45°) = 0.707
z = 1 (TÁVOLSÁG, nem 0!)
```

#### 4. Súlyponti pont (Centroid) számítása

```python
# Súlyponti koordináták összegzése (az előző ciklusban)
x += x_temp
y += y_temp
z += z_temp
valid_points += 1

# ... majd után:

# Átlag (súlyponti pont)
x_avg = x / valid_points
y_avg = y / valid_points
z_avg = z / valid_points
```

A **centroid** (súlyponti pont) az összes pont átlaga:

$$x_{centroid} = \frac{1}{N} \sum_{i=1}^{N} x_i$$

$$y_{centroid} = \frac{1}{N} \sum_{i=1}^{N} y_i$$

$$z_{centroid} = \frac{1}{N} \sum_{i=1}^{N} z_i$$

Ez a pont képviseli a pont felhő "közepét".

#### 5. PointCloud2 létrehozása

```python
cloud = pcl2.create_cloud_xyz32(msg.header, newPoint)
self.pub_points.publish(cloud)
```

A `sensor_msgs_py.point_cloud2` modul segítségével egy PointCloud2 üzenetet hozunk létre:
- `msg.header`: az eredeti LaserScan fejléce (frame_id, timestamp)
- `newPoint`: a pont lista (3D koordináták)

#### 6. Marker vizualizáció

```python
marker = Marker()
marker.header = msg.header
marker.ns = "weight_point"
marker.id = 0
marker.type = marker.SPHERE
marker.action = marker.ADD

# Pozíció: súlyponti pont
marker.pose.position.x = x_avg
marker.pose.position.y = y_avg
marker.pose.position.z = z_avg
marker.pose.orientation.w = 1.0

# Méret: 10 cm átmérő
marker.scale.x = 0.1
marker.scale.y = 0.1
marker.scale.z = 0.1

# Szín: zöld (RGBA)
marker.color.a = 1.0
marker.color.r = 0.0
marker.color.g = 1.0
marker.color.b = 0.0

# Lifetime: 0.1 sec (folyamatosan frissül)
marker.lifetime = Duration(seconds=0.1).to_msg()
self.pub_marker.publish(marker)
```

Ez egy **zöld gömb** a RViz-ben a súlyponti pont helyén. A lifetime 0.1 másodperc, ami azt jelenti, hogy ha nem frissítik 0.1 másodpercen belül, eltűnik.

#### 7. TF transzformáció: lokális → globális

```python
# Lokális PoseStamped (scan frame-ben)
pose_local = PoseStamped()
pose_local.header = msg.header
pose_local.pose.position.x = x_avg
pose_local.pose.position.y = y_avg
pose_local.pose.position.z = z_avg
pose_local.pose.orientation.w = 1.0

# TF lekérdezése: scan → map
if self.tf_buffer.can_transform("map", msg.header.frame_id, msg.header.stamp, Duration(seconds=0.1)):
    trans_scan2map = self.tf_buffer.lookup_transform(
        "map", 
        msg.header.frame_id, 
        msg.header.stamp
    )
    
    # Transzformáció alkalmazása
    pose_map = tf2_geometry_msgs.do_transform_pose(pose_local, trans_scan2map)
    
    # Globális póz publikálása
    self.pub_pose.publish(pose_map)
```

**Mi történik itt?**

A súlyponti pont a lézer szenor keretében van kiszámítva (pl. `base_scan`). De azt akarjuk, hogy a `map` (világkoordinátarendszer) keretében legyen.

```
map (világkoordináta)
 └── transzformáció: [1 m előre, 45° fordulva]
      └── base_scan (lézer szenzor)
           └── súlyponti pont (0.5, 0.2) lokálisan
                → TF transzformáció után → (1.5, 0.7) globálisan
```

A `do_transform_pose` ezt végzi el: a pontot a transzformáció szerint megváltoztatja.

---

## Launch fájl (`gyak12.launch.xml`)

### Mit csinál?

```xml
<!-- Rosbag lejátszása (szimulációs szenzoradatok) -->
<executable
    cmd="ros2 bag play --clock 1000 $(find-pkg-share gyak3)/bag"
    output="screen"
    shell="true"
/>
```

Ez a rosbag fájlt játssza le, amely a szimulált szenzoradatokat tartalmazza.

```xml
<!-- Node1: odometria feldolgozás -->
<node pkg="gyak12" exec="node1" name="node1" output="screen" />

<!-- Node2: LaserScan feldolgozás -->
<node pkg="gyak12" exec="node2" name="node2" output="screen" />

<!-- RViz -->
<node pkg="rviz2" exec="rviz2" name="rviz2" args="-d $(find-pkg-share gyak12)/rviz/gyak12.rviz"/>
```

Mindkét node-ot és az RViz-t egy paranccsal indítja.

---

## RViz konfiguráció (`gyak12.rviz`)

### Megjelenítés

1. **Grid**: 10x10 méteres rács
2. **LaserScan** (`/scan`): fehér pontok (lézer mérések)
3. **PointCloud2** (`/cloud`): a Descartes-koordinátákra konvertált pontok
4. **Marker** (`/marker`): zöld gömb a súlyponti pontban
5. **Odometry** (`/poz`): a robot pozíciója és orientációja

**Fixed Frame**: `map` vagy `base_scan` (az actual szenzor kerete)

---

## Futtatás és működés

### Előfeltételések

```bash
# Workspace építése
cd ~/codes/mgm/mgm
colcon build --packages-select gyak12 gyak3

# Beállítás
source install/setup.bash
```

### Indítás

```bash
# Launch fájl futtatása
ros2 launch gyak12 gyak12.launch.xml
```

Ez a következőket indítja:
1. Rosbag lejátszást (szimulált szenzoradatok)
2. Node1-et (odometria feldolgozás)
3. Node2-t (LaserScan feldolgozás)
4. RViz-et

### Mit látsz az RViz-ben?

- **Fehér pontok**: LaserScan mérések (eredeti poláris)
- **Többszínű pontok**: a `/cloud` topic (Descartes, z=távolság)
- **Zöld gömb**: súlyponti pont
- **Nyíl**: robot orientációja (odometriából)

### Ellenőrzés

Külön terminálban:

```bash
# Node1 topicjainak ellenőrzése
ros2 topic echo /elmozdulas --once
ros2 topic echo /poz --once | head -20

# Node2 topicjainak ellenőrzése
ros2 topic echo /cloud --once | head -20
ros2 topic echo /marker --once
ros2 topic echo /pose --once
```

---

## Vizsga elemei és pontozás

### Node1 (8 pont összesen)

1. **Node létrehozása** (1 pont)
   - Feliratkozik az `/odom` topicra
   - Kiszámolja az elmozdulást

2. **Elmozdulás publikálása** (1 pont)
   - Float64 az `/elmozdulas` topicra
   - Összes elmozdulás összegzése

3. **Feltételes odometria publikálása** (2 pont)
   - Csak akkor publikál, ha az egész méterek száma nő
   - Odometry üzenet az `/poz` topicra

4. **Launch fájl** (2 pont)
   - Node1 indítása
   - RViz automatikus indítása

5. **Kód minősége** (2 pont)
   - Kommentek, érthető logika

### Node2 (11 pont összesen)

1. **Node létrehozása** (1 pont)
   - Feliratkozik a `/scan` topicra

2. **Poláris → Descartes konverzió** (1 pont)
   - x = r * cos(θ)
   - y = r * sin(θ)
   - z = r (távolság)

3. **PointCloud2 publikálása** (3 pont)
   - Minden valid pont a `/cloud` topicra
   - Helyes header és frame_id

4. **Súlyponti pont számítása** (3 pont)
   - Marker (zöld gömb) az `/marker` topicra
   - 10 cm átmérő, helyes pozíció

5. **TF transzformáció** (3 pont)
   - scan → map transzformáció
   - PoseStamped az `/pose` topicra

6. **Launch fájl** (2 pont)
   - Node2 indítása
   - RViz automatikus indítása

---

## Főbb tanulságok

### 1. **Euclidean távolság**
Síkmozgás esetén: $d = \sqrt{(x_2 - x_1)^2 + (y_2 - y_1)^2}$

### 2. **Feltételes logika**
`floor()` függvény használatával könnyű az egész értéknél történő változást detektálni.

### 3. **Poláris → Descartes**
Lézerszenzorok poláris koordinátában adják az adatokat; konvertálni kell.

### 4. **Centroid**
Pont felhő középpontja: összes koordináta átlaga.

### 5. **TF transzformáció**
Szenzor keretből világkeretbe: kritikus a robotika-ban.

### 6. **PointCloud2**
Nagy pont halmazok ROS-ban történő kezeléséhez standard üzenet.

---

## Gyakori hibák és megoldások

### "No transformation between frames"
- Ellenőrizd: a rosbag tartalmaz-e TF adatokat?
- RViz: állítsd a fixed frame-et az aktuális keretre (pl. `base_scan`)

### PointCloud2 üres
- Ellenőrizd: a LaserScan tartalmaz-e valid méréseket?
- Szűrés: `range_min` és `range_max` közötti tartomány?

### Zöld gömb nem mozdul
- Ellenőrizd: a `/marker` topic publikálódik-e?
- RViz: Add hozzá a `Marker` display-t

### Node1 nem publikál `/poz`-t
- Ellenőrizd: az odometria mozdul-e?
- Debug: print-elj az egész méterek számát

### Túl sok vagy túl kevés pont a felhőben
- Ellenőrizz `range_min` és `range_max` értékeket
- Vagy: a lézer szenzor beállítása

---

## Továbbfejlesztési ötletek

1. **Statisztikák**: min/max távolság, pontok száma
2. **Szűrés**: csak egy bizonyos távolsági tartomány
3. **Clustering**: pont csoportok felismerése
4. **Sebesség becslése**: odometriához alapozva
5. **Kalman filter**: kisebb zaj az odometriában

---

## Hasznos parancsok

```bash
# Topic lista
ros2 topic list

# Topic típusai
ros2 topic list -t

# Topic adatok
ros2 topic echo /elmozdulas
ros2 topic echo /cloud --once | head -50

# Node info
ros2 node list
ros2 node info /zh_node

# TF fa
ros2 run tf2_tools view_frames.py
dot -Tpng frames.gv -o frames.png

# Rosbag info
ros2 bag info $(find-pkg-share gyak3)/bag
```

---

## Referenciák

- [ROS2 nav_msgs/Odometry](https://docs.ros2.org/latest/api/nav_msgs/msg/Odometry.html)
- [ROS2 sensor_msgs/LaserScan](https://docs.ros2.org/latest/api/sensor_msgs/msg/LaserScan.html)
- [ROS2 sensor_msgs/PointCloud2](https://docs.ros2.org/latest/api/sensor_msgs/msg/PointCloud2.html)
- [TF2](https://docs.ros.org/en/humble/Concepts/Intermediate/Tf2/Tf2.html)
- [Pontfelhő feldolgozás](https://github.com/ros-perception/point_cloud_to_laserscan)
- [Centroid](https://en.wikipedia.org/wiki/Centroid)

---

**Készült**: 2025. december  
**ROS verzió**: ROS 2 Humble  
**Tesztelve**: ZH vizsga feladat (Zhk) szimulációban  
**Pontszám**: 19 pont (8 pont Node1 + 11 pont Node2)
