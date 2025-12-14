# gyak6 – LaserScan feldolgozás és vizualizáció

A `gyak6` csomag bemutatja, hogyan dolgozz fel **LaserScan** (LiDAR) adatokat ROS 2-ben. A node átalakítja a poláris koordinátájú méréseket Descartes-koordinátákká, publikálja őket **PointCloud2** és **MarkerArray** formátumban, valamint megkeresi és kiemeli a **legközelebbi pontot**.

---

## 🎯 Mi a cél?

- **LaserScan** adatok fogadása és feldolgozása (tipikus érzékelő robotokban).
- **Koordináta-transzformáció**: poláris (távolság, szög) → Descartes (x, y, z).
- **Szűrés**: csak valid tartományban lévő mérések feldolgozása.
- **PointCloud2**: 3D ponthalmaz publikálása (sensor adatok reprezentációja).
- **Marker vizualizáció**: SPHERE_LIST típusú marker a scan pontokhoz.
- **Legközelebbi pont**: a legkisebb távolságú mérés kiemelése és publikálása.

---

## 📁 Fájlstruktúra

```
mgm_gyak/gyak6/
├── package.xml                   # Csomag metaadatok (ament_python)
├── setup.py                      # Python entry points (test_scan futtatható)
├── setup.cfg                     # Python setuptools konfig
├── gyak6/
│   ├── __init__.py
│   └── test_scan.py             # ScanHandler node: LaserScan feldolgozás
├── launch/
│   └── gyak6.launch.xml         # Rosbag play + node + RViz indítás
├── rviz/
│   └── gyak6.rviz               # RViz konfig (LaserScan, PointCloud2, MarkerArray, Pose)
├── resource/
│   └── gyak6
└── test/                        # Tesztek
```

Főbb fájlok:
- [gyak6/test_scan.py](gyak6/test_scan.py) – LaserScan feldolgozó node
- [launch/gyak6.launch.xml](launch/gyak6.launch.xml) – indító fájl
- [rviz/gyak6.rviz](rviz/gyak6.rviz) – RViz beállítások

---

## 🔧 A ScanHandler node működése

### Áttekintés

[gyak6/test_scan.py](gyak6/test_scan.py)

A `ScanHandler` node feliratkozik a `/scan` topicra és minden beérkező LaserScan üzenetből:
1. **Szűri** a valid méréseket (range_min és range_max között).
2. **Átalakítja** poláris koordinátákból Descartes-koordinátákba.
3. **Publikálja**:
   - `/cloud` – PointCloud2 (3D ponthalmaz)
   - `/viz` – MarkerArray (SPHERE_LIST marker)
   - `/closest_point` – PoseStamped (legközelebbi pont)

---

### Mi az a LaserScan?

A **LaserScan** üzenet (`sensor_msgs/LaserScan`) LiDAR/LIDAR szenzorok adatait reprezentálja:

```python
header              # Időbélyeg és koordinátarendszer (pl. "base_scan")
angle_min           # Minimum szög (rad) - hol kezdődik a mérés
angle_max           # Maximum szög (rad) - hol végződik a mérés
angle_increment     # Szöglépés (rad) - mérések közötti szög
time_increment      # Időlépés (sec) - mérések közötti idő
scan_time           # Teljes scan ideje (sec)
range_min           # Minimum valid távolság (m)
range_max           # Maximum valid távolság (m)
ranges[]            # Távolságmérések (m) - egy tömb, minden elem egy irány
intensities[]       # Intenzitások (opcionális)
```

**Példa:** ha `angle_min = -π/2`, `angle_max = π/2`, `angle_increment = π/180`, akkor 180 mérés van -90°-tól +90°-ig, 1°-onként.

---

### Részletes kód magyarázat

#### Inicializálás

```python
class ScanHandler(Node):
    def __init__(self):
        super().__init__('test_viz')
        
        self.pub_cloud = self.create_publisher(PointCloud2, "/cloud", 1)
        self.pub_marker = self.create_publisher(MarkerArray, "/viz", 1)
        self.closest_point_pub = self.create_publisher(PoseStamped, "/closest_point", 1)
        
        self.sub = self.create_subscription(LaserScan, "/scan", self.callback_scan, 1)
```

**Három publisher:**
1. `/cloud` – PointCloud2 (strukturált 3D adatok)
2. `/viz` – MarkerArray (vizuális reprezentáció)
3. `/closest_point` – PoseStamped (legközelebbi pont pozíciója)

---

#### LaserScan callback – mérések feldolgozása

```python
def callback_scan(self, scan_in: LaserScan):
    newPoint = []
    
    for i in range(len(scan_in.ranges)):
        # Szűrés: csak valid mérések
        if ((scan_in.range_min < scan_in.ranges[i]) and (scan_in.ranges[i] < scan_in.range_max)):
            # Szög kiszámítása
            angle = scan_in.angle_min + scan_in.angle_increment * i
            
            # Poláris → Descartes transzformáció
            x = scan_in.ranges[i] * math.cos(angle)
            y = scan_in.ranges[i] * math.sin(angle)
            z = 0.0
            
            newPoint.append([x, y, z])
```

**Magyarázat lépésenként:**

1. **Iterálás**: végigmegyünk az összes mérésen (`ranges` tömb).

2. **Szűrés**: csak a valid tartományban lévő mérések (túl közel/túl távoli kiszűrése).
   - `range_min`: pl. 0.1 m – ennél közelebbi mérés nem valid (min távolság)
   - `range_max`: pl. 10.0 m – ennél távolabbi mérés nem valid (max távolság)
   - Az ezek közé eső mérések megbízhatóak.

3. **Szög kiszámítása**: 
   ```
   angle = angle_min + i × angle_increment
   ```
   - `i = 0`: `angle_min` (bal szélső irány)
   - `i = len(ranges)-1`: `angle_max` (jobb szélső irány)

4. **Koordináta-transzformáció (poláris → Descartes)**:
   ```
   x = r × cos(θ)
   y = r × sin(θ)
   z = 0
   ```
   - **Poláris koordináták**: (távolság, szög) – ahogy a LiDAR méri
   - **Descartes koordináták**: (x, y, z) – ahogy a térbeli pozíciót reprezentáljuk
   - `z = 0.0`: síkbeli scan (2D LiDAR esetén)

**Vizuális magyarázat:**
```
        y ↑
          |
    θ=90° |
          |
    •-----+-----• x →
  θ=180°  |     θ=0°
          |
    θ=-90°|
```

---

#### MarkerArray publikálás – SPHERE_LIST

```python
marker_array_ = MarkerArray()

marker = Marker()
marker.header = scan_in.header
marker.ns = "scan"
marker.id = 0
marker.type = Marker.SPHERE_LIST  # Több gömb egyetlen markerben
marker.action = Marker.ADD

marker.scale.x = 0.05
marker.scale.y = 0.05
marker.scale.z = 0.05

marker.color.a = 1.0
marker.color.b = 1.0  # Kék

for point in newPoint:
    p = Point()
    p.x = point[0]
    p.y = point[1]
    p.z = point[2]
    marker.points.append(p)

marker.lifetime = rclpy.duration.Duration(seconds=0.15).to_msg()

marker_array_.markers.append(marker)
self.pub_marker.publish(marker_array_)
```

**SPHERE_LIST marker:**
- **Hatékony**: egyetlen marker objektum, de több gömböt tartalmaz.
- Minden pont a `marker.points` listába kerül.
- Minden gömb ugyanolyan méretű és színű (közös `scale` és `color`).

**Előny a hagyományos SPHERE-hez képest:**
- 100 pont → 1 marker (gyors), nem 100 marker (lassú).

---

#### PointCloud2 publikálás

```python
from sensor_msgs_py import point_cloud2 as pcl2

localCloud = pcl2.create_cloud_xyz32(scan_in.header, newPoint)
self.pub_cloud.publish(localCloud)
```

**PointCloud2:**
- Strukturált 3D ponthalmaz üzenet.
- `create_cloud_xyz32`: segédfüggvény, amely XYZ float32 formátumú pontfelhőt készít.
- **Használat**: perception algoritmusok (pl. objektumdetektálás, térképépítés).

**Különbség a MarkerArray-hez képest:**
- PointCloud2: nyers sensor adat (algoritmusok számára).
- MarkerArray: vizualizáció (emberek számára).

---

#### Legközelebbi pont keresése

```python
closest_dist = 1000  # Nagy kezdőérték
closest_dist_angle = 0

for i in range(len(scan_in.ranges)):
    dist = scan_in.ranges[i]
    if dist < closest_dist:
        closest_dist = dist
        closest_dist_angle = scan_in.angle_min + i * scan_in.angle_increment
```

**Egyszerű minimum keresés:**
- Végigmegyünk az összes mérésen.
- Megkeressük a legkisebb távolságot és a hozzá tartozó szöget.

```python
closest_point = PoseStamped()
closest_point.header = scan_in.header

closest_point.pose.position.x = closest_dist * math.cos(closest_dist_angle)
closest_point.pose.position.y = closest_dist * math.sin(closest_dist_angle)
closest_point.pose.position.z = 0.0
closest_point.pose.orientation.w = 1.0

self.closest_point_pub.publish(closest_point)
```

**PoseStamped:**
- Pozíció (x, y, z) + orientáció (quaternion).
- `orientation.w = 1.0`: nincs forgatás (egység quaternion).

**Használat:**
- Ütközéselkerülés: reagálás a legközelebbi akadályra.
- Követés: objektum/fal követése.

---

## 🚀 Launch fájl

[launch/gyak6.launch.xml](launch/gyak6.launch.xml)

```xml
<launch>
    <!-- Rosbag play (gyak3 bag-ből) -->
    <executable
        cmd="ros2 bag play --clock 1000 $(find-pkg-share gyak3)/bag"
        output="screen"
        name="rosbag_play"
        shell="true"
    />

    <!-- ScanHandler node -->
    <node pkg="gyak6" exec="test_scan" name="scan_visualization" output="screen"/>

    <!-- RViz -->
    <node pkg="rviz2" exec="rviz2" name="rviz2" args="-d $(find-pkg-share gyak6)/rviz/gyak6.rviz"/>
</launch>
```

**Mit indít?**
1. Rosbag lejátszás (gyak3 bag: `/odom`, `/scan`, stb.)
2. `test_scan` node (LaserScan feldolgozás)
3. RViz előre beállított nézettel

---

## 🖼️ RViz beállítások

[rviz/gyak6.rviz](rviz/gyak6.rviz)

- Fix Frame: `odom`
- Megjelenítők:
  - `Grid` – referencia rács
  - `Odometry` – `/odom` (robot pozíció)
  - **`LaserScan`** – `/scan` (eredeti LiDAR adatok, fehér pontok)
  - **`PointCloud2`** – `/cloud` (transzformált pontfelhő)
  - **`MarkerArray`** – `/viz` (kék gömbök)
  - **`Pose (ClosestPoint)`** – `/closest_point` (narancssárga nyíl)
  - `RobotModel` – robot vizualizáció

**Összehasonlítás:**
- `LaserScan` display: beépített RViz megjelenítő (egyszerű).
- `PointCloud2` display: ugyanaz az adat, más formátumban.
- `MarkerArray` display: egyedi vizualizáció (testreszabható).

---

## 📦 Build és futtatás

```bash
# Build
colcon build --packages-select gyak6

# Forrásold a környezetet
source install/setup.bash

# Indítás
ros2 launch gyak6 gyak6.launch.xml
```

**Mit látsz RViz-ben?**
- Fehér pontok: LaserScan (eredeti mérések).
- Kék gömbök: MarkerArray (SPHERE_LIST).
- Kis fehér pontok: PointCloud2 (XYZ adatok).
- Narancssárga nyíl: legközelebbi pont.

**Ellenőrzés:**
```bash
ros2 topic list
ros2 topic echo /scan
ros2 topic echo /cloud
ros2 topic echo /closest_point

ros2 topic hz /scan  # Mérési frekvencia (pl. 10 Hz)
```

---

## 🎓 Mit tanulsz ebből?

### 1. LaserScan alapok
- Poláris koordináták: (távolság, szög).
- `angle_min`, `angle_max`, `angle_increment` – szögek kiszámítása.
- `range_min`, `range_max` – valid mérések szűrése.

### 2. Koordináta-transzformáció
- **Poláris → Descartes**: `x = r·cos(θ)`, `y = r·sin(θ)`.
- Matematikai alapok: trigonometria.

### 3. PointCloud2
- 3D ponthalmaz reprezentáció.
- `sensor_msgs_py.point_cloud2` – segédfüggvények.
- `create_cloud_xyz32` – XYZ float32 formátum.

### 4. SPHERE_LIST marker
- Hatékony vizualizáció sok pont esetén.
- Egy marker, több gömb.

### 5. Sensor adat feldolgozás
- Szűrés: érvénytelen mérések kiszűrése.
- Minimum keresés: legközelebbi pont.
- Valós idejű feldolgozás: callback pattern.

---

## 🔍 Gyakori hibák

| Probléma | Ok | Megoldás |
|---|---|---|
| Nincs `/scan` adat | Rosbag nem fut vagy nincs LiDAR | Ellenőrizd a rosbag-et vagy szenzort |
| PointCloud üres | Minden mérés invalid (range túl nagy/kicsi) | Ellenőrizd a range_min/max értékeket |
| Marker nem látszik | RViz display hiányzik vagy rossz topic | Adj hozzá MarkerArray display-t `/viz`-zel |
| Koordináták rosszak | Szög radiánban van, nem fokban | Használj `math.cos/sin` radiánnal |
| Legközelebbi pont rossz | Nem szűrtél érvényes méréseket | Szűrd a range_min/max alapján |

---

## 💡 Gyakorlási ötletek

1. **Szűrési tartomány módosítása**: csak előre nézz (`-π/4` és `+π/4` között).
   ```python
   if (angle > -math.pi/4) and (angle < math.pi/4):
       # feldolgozás
   ```

2. **Távolság-alapú színezés**: közelebbi pontok pirosak, távolabbiak zöldek.
   ```python
   marker.colors.append(...)  # Color objektumok hozzáadása
   ```

3. **Cluster keresés**: csoportosítsd a közeli pontokat.

4. **Fal detektálás**: LINE_STRIP markerrel vonalak rajzolása.

5. **Statisztikák**: átlagos távolság, minimum, maximum számítása.

---

## 📚 Hasznos ROS2 parancsok

```bash
# LaserScan adatok részletei
ros2 topic info /scan
ros2 topic hz /scan
ros2 interface show sensor_msgs/msg/LaserScan

# PointCloud2 részletei
ros2 interface show sensor_msgs/msg/PointCloud2

# Adatok mentése
ros2 bag record /scan /odom /cloud

# Csak a legközelebbi pont követése
ros2 topic echo /closest_point
```

---

## 🎯 Vizsgára felkészülés

### Amit tudnod kell:
- ✅ Mi az a LaserScan és hogyan strukturált?
- ✅ Hogyan számítsd ki a szöget egy adott méréshez?
- ✅ Poláris → Descartes transzformáció képlete?
- ✅ Mi a PointCloud2 és mire használjuk?
- ✅ Különbség a LaserScan, PointCloud2 és MarkerArray között?
- ✅ Hogyan keress minimum/maximum értéket?
- ✅ Miért kell szűrni a méréseket?

### Vizsgán gyakori feladatok:
- LaserScan adatok feldolgozása (koordináta-transzformáció).
- Szűrés megadott szögtartományra.
- Legközelebbi/legtávolabbi pont keresése.
- PointCloud publikálás.
- Marker vizualizáció készítése.

---

## 🏁 Összefoglalás

A `gyak6` csomag a LiDAR/LaserScan adatok feldolgozását mutatja be. A poláris koordinátákból Descartes-koordináták készítése, a valid mérések szűrése és a különböző reprezentációk (LaserScan, PointCloud2, MarkerArray) publikálása alapvető készségek a robot perception területén. A legközelebbi pont keresése pedig egy egyszerű, de hasznos példa az ütközéselkerülésre és objektumkövetésre.

**Sok sikert a tanuláshoz!** 🚀
