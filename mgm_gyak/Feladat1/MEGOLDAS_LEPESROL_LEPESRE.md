# Feladat1 - Megoldás lépésről lépésre

## Áttekintés

Ez a dokumentum lépésről-lépésre bemutatja a Feladat1 megoldását, magyarázatokkal és ZH tippekkel.

## Feladat követelmények

1. **Package létrehozása**: „proba1" nevű ROS package
2. **Odometry feldolgozás**: Feliratkozás `/agent1/odom/ground_truth`-ra, yaw szög számítása
3. **Path feldolgozás**: Feliratkozás `/path`-ra, legközelebbi pont keresése
4. **Closest point publikálás**: `/closest_point` topicra (geometry_msgs/PoseStamped)
5. **Szögszámítás**: Legközelebbi pont és pozíció közötti szög - yaw
6. **Szög publikálás**: std_msgs/Float64 típusú üzenet
7. **Launch file**: Node és RViz indítása
8. **RViz konfig**: LIDAR és legközelebbi pont megjelenítése

---

## LÉPÉS 1: Package struktúra létrehozása

### Célok
- ROS 2 Python package alapvető struktúrájának felállítása
- Függőségek deklarálása
- Entry point beállítása

### Fájlok

#### 1.1 Könyvtár struktúra

```
Feladat1/
└── proba1/
    ├── package.xml          # Package metadata
    ├── setup.py             # Python setup
    ├── setup.cfg            # Setup konfiguráció
    ├── resource/            # Marker könyvtár
    │   └── proba1
    ├── proba1/              # Forráskód
    │   ├── __init__.py
    │   └── proba1_node.py
    ├── launch/              # Launch fájlok
    │   └── proba1.launch.xml
    └── rviz/                # RViz konfigok
        └── proba1.rviz
```

#### 1.2 package.xml - Függőségek

```xml
<depend>rclpy</depend>              <!-- ROS 2 Python könyvtár -->
<depend>nav_msgs</depend>           <!-- Odometry, Path -->
<depend>geometry_msgs</depend>      <!-- PoseStamped -->
<depend>std_msgs</depend>           <!-- Float64 -->
<depend>tf_transformations</depend> <!-- Quaternion → Euler -->
<depend>tf2_ros</depend>            <!-- TF támogatás -->
```

**ZH tipp:** Mindig add hozzá az összes üzenettípus package-ét a függőségekhez!

#### 1.3 setup.py - Entry point

```python
entry_points={
    'console_scripts': [
        'proba1_node = proba1.proba1_node:main',
    ],
}
```

Ez teszi lehetővé: `ros2 run proba1 proba1_node`

---

## LÉPÉS 2: Node váz elkészítése

### Célok
- Node osztály létrehozása
- Állapotváltozók inicializálása
- Subscriber és Publisher setup

### 2.1 Importok

```python
import rclpy
from rclpy.node import Node
import math

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
from tf_transformations import euler_from_quaternion
```

**ZH tipp:** `tf_transformations` kell a quaternion → euler konverzióhoz!

### 2.2 Állapotváltozók

```python
class Proba1Node(Node):
    def __init__(self):
        super().__init__('proba1_node')
        
        # Saját állapot (Odometry-ből)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0  # radiánban
        
        # Path tárolása
        self.path = None
```

**Miért kellenek?**
- `current_x, y`: Legközelebbi pont kereséséhez kell a saját pozíció
- `current_yaw`: Szögszámításhoz kell
- `path`: Path üzenet későbbi feldolgozásához

### 2.3 Subscribers

```python
# Odometry feliratkozó
self.sub_odom = self.create_subscription(
    Odometry,                      # Üzenet típus
    '/agent1/odom/ground_truth',   # Topic név (FELADAT SPECIFIKÁCIÓ!)
    self.callback_odom,            # Callback függvény
    10                             # QoS queue
)

# Path feliratkozó
self.sub_path = self.create_subscription(
    Path,
    '/path',
    self.callback_path,
    10
)
```

**ZH tipp:** A topic neveket PONTOSAN a feladat szerint írd!

### 2.4 Publishers

```python
# Legközelebbi pont publikálása
self.pub_closest = self.create_publisher(
    PoseStamped,
    '/closest_point',  # Feladat specifikáció
    10
)

# Szögkülönbség publikálása
self.pub_angle = self.create_publisher(
    Float64,
    '/angle_diff',     # Saját választás (feladat nem specifikus)
    10
)
```

---

## LÉPÉS 3: Odometry feldolgozás (yaw számítás)

### Célok
- Odometry üzenet fogadása
- Pozíció kinyerése
- Quaternion → Euler (yaw) konverzió

### 3.1 Callback implementáció

```python
def callback_odom(self, msg: Odometry):
    # Pozíció
    self.current_x = msg.pose.pose.position.x
    self.current_y = msg.pose.pose.position.y
    
    # Orientáció: quaternion
    q = [
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w
    ]
    
    # Euler konverzió
    (roll, pitch, yaw) = euler_from_quaternion(q)
    self.current_yaw = yaw  # radiánban: -pi..pi
    
    # Ha van path, feldolgozzuk
    if self.path is not None:
        self.process_path()
```

### 3.2 Quaternion → Euler magyarázat

**Quaternion:** 4D reprezentáció (x, y, z, w) - nincs gimbal lock
**Euler szögek:** 3D rotáció (roll, pitch, yaw) - intuitívabb

```
roll:  x tengely körüli forgatás (dőlés oldalra)
pitch: y tengely körüli forgatás (dőlés előre/hátra)
yaw:   z tengely körüli forgatás (fordulás vízszintesen) ← EZ KELL!
```

**ZH tipp:** `euler_from_quaternion()` tuple-t ad vissza: `(roll, pitch, yaw)`

---

## LÉPÉS 4: Path feldolgozás (legközelebbi pont)

### Célok
- Path üzenet fogadása és tárolása
- Legközelebbi pont keresése (euklideszi távolság)

### 4.1 Path callback

```python
def callback_path(self, msg: Path):
    self.path = msg
    self.get_logger().info(f'Path érkezett: {len(self.path.poses)} pont')
    
    # Feldolgozás, ha van odometry
    self.process_path()
```

### 4.2 Legközelebbi pont keresés

```python
def process_path(self):
    # Biztonsági ellenőrzések
    if self.path is None or len(self.path.poses) == 0:
        return
    
    # Legközelebbi pont keresése
    closest_distance = float('inf')  # végtelen kezdőérték
    closest_index = 0
    
    for i, pose_stamped in enumerate(self.path.poses):
        # Euklideszi távolság
        dx = pose_stamped.pose.position.x - self.current_x
        dy = pose_stamped.pose.position.y - self.current_y
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance < closest_distance:
            closest_distance = distance
            closest_index = i
    
    # Legközelebbi pont
    closest_pose = self.path.poses[closest_index]
    
    # Publikálás
    self.pub_closest.publish(closest_pose)
```

**Algoritmus:**
1. Végigmegyünk a path összes pontján
2. Kiszámoljuk a távolságot mindegyiktől
3. A legkisebb távolságú pont a legközelebbi

**ZH tipp:** `float('inf')` hasznos kezdőérték minimumkeresésnél!

---

## LÉPÉS 5: Szögszámítás

### Célok
- Legközelebbi pont irányának meghatározása (map x tengelyhez képest)
- Yaw kivonása
- Normalizálás -π..π tartományba

### 5.1 Implementáció

```python
# Delta koordináták a legközelebbi pontig
dx = closest_pose.pose.position.x - self.current_x
dy = closest_pose.pose.position.y - self.current_y

# Irány szöge (map x tengelyhez képest)
angle_to_closest = math.atan2(dy, dx)

# Szögkülönbség: cél irány - saját orientáció
angle_diff = angle_to_closest - self.current_yaw

# Normalizálás -pi..pi tartományba
angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

# Publikálás
angle_msg = Float64()
angle_msg.data = angle_diff
self.pub_angle.publish(angle_msg)
```

### 5.2 Geometria magyarázat

```
       Y
       ^
       |
   P   |
   *   |      * = Legközelebbi pont (xp, yp)
   |   |      R = Robot pozíció (xr, yr)
   | / |      θ = angle_to_closest
   |/  |      ψ = current_yaw (robot orientáció)
   R---+-------> X
       |
```

**atan2(dy, dx):**
- Megadja a vektor szögét az x tengelyhez képest
- Tartomány: -π..π (teljes 360°)
- Előjeles: pozitív = felfelé, negatív = lefelé

**Szögkülönbség:**
- Ha pozitív → balra kell fordulni
- Ha negatív → jobbra kell fordulni
- Ha ~0 → egyenesen előre

**ZH tipp:** 
- Mindig `atan2(y, x)` sorrendben! (NEM `atan2(x, y)`)
- Szög normalizálás: `atan2(sin(θ), cos(θ))` → garantáltan -π..π

---

## LÉPÉS 6: Launch file készítése

### Célok
- Node automatikus indítása
- RViz automatikus indítása
- `use_sim_time` beállítása (bag lejátszáshoz)

### 6.1 proba1.launch.xml

```xml
<launch>
  <!-- Node indítása -->
  <node pkg="proba1" exec="proba1_node" name="proba1_node" output="screen">
    <param name="use_sim_time" value="true"/>
  </node>

  <!-- RViz indítása konfigurációval -->
  <node pkg="rviz2" exec="rviz2" name="rviz2" 
        args="-d $(find-pkg-share proba1)/rviz/proba1.rviz">
    <param name="use_sim_time" value="true"/>
  </node>
</launch>
```

**Magyarázat:**
- `pkg="proba1"`: Package név
- `exec="proba1_node"`: Entry point név (setup.py-ból)
- `name="proba1_node"`: Node neve a ROS gráfban
- `output="screen"`: Log-ok terminálra
- `use_sim_time="true"`: Bag file időt használja (NEM rendszer időt)
- `$(find-pkg-share proba1)`: Package install könyvtára

**ZH tipp:** `use_sim_time` KRITIKUS bag lejátszáshoz!

---

## LÉPÉS 7: RViz konfiguráció

### Célok
- LIDAR megjelenítése (LaserScan)
- Legközelebbi pont megjelenítése (PoseStamped)
- Path megjelenítése
- Odometry megjelenítése

### 7.1 Alapbeállítások

```yaml
Global Options:
  Fixed Frame: map      # Referencia koordinátarendszer
  Frame Rate: 30        # Frissítés gyakorisága
```

**Fixed Frame:** Minden elem ehhez képest jelenik meg!

### 7.2 Display-k

#### LaserScan (LIDAR)
```yaml
- Class: rviz_default_plugins/LaserScan
  Topic: /scan
  Size (Pixels): 3
  Style: Flat Squares
  Color Transformer: Intensity
```

#### PoseStamped (Legközelebbi pont)
```yaml
- Class: rviz_default_plugins/PoseStamped
  Topic: /closest_point
  Shape: Arrow
  Color: 255; 25; 0  # Piros/narancs
  Axes Length: 0.5
```

#### Path
```yaml
- Class: rviz_default_plugins/Path
  Topic: /path
  Line Style: Lines
  Color: 25; 255; 0  # Zöld
```

#### Odometry
```yaml
- Class: rviz_default_plugins/Odometry
  Topic: /agent1/odom/ground_truth
  Shape: Arrow
  Keep: 100  # Utolsó 100 pozíció
```

**ZH tipp:** RViz config fájlt nem kell kézzel írni - RViz-ben állítsd be, majd "Save Config As"!

---

## LÉPÉS 8: Build és tesztelés

### 8.1 Build

```bash
# Workspace gyökerében
cd ~/ros2_ws
colcon build --packages-select proba1 --symlink-install
source install/setup.bash
```

**--symlink-install:** Python kód módosítása után nem kell rebuild!

### 8.2 Futtatás

```bash
# Launch file (node + RViz)
ros2 launch proba1 proba1.launch.xml
```

### 8.3 Bag lejátszás (külön terminál)

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 bag play /path/to/bagfile --clock
```

**--clock:** Szimulált óra publikálása (/clock topic)

### 8.4 Ellenőrzés

```bash
# Topicok
ros2 topic list
ros2 topic echo /closest_point
ros2 topic echo /angle_diff

# Node info
ros2 node info /proba1_node

# Topic frekvencia
ros2 topic hz /agent1/odom/ground_truth
```

---

## Teljes működési folyamat

```
1. Bag lejátszás indul
   ↓
2. /agent1/odom/ground_truth érkezik
   ↓ callback_odom()
3. Pozíció (x, y) és yaw tárolása
   ↓
4. /path érkezik (egyszer vagy periodikusan)
   ↓ callback_path()
5. Path tárolása
   ↓
6. process_path() hívódik (odom vagy path callback-ből)
   ↓
7. Legközelebbi pont keresése (for loop, min távolság)
   ↓
8. Legközelebbi pont publikálása (/closest_point)
   ↓
9. Szögszámítás (atan2, normalizálás)
   ↓
10. Szög publikálása (/angle_diff)
    ↓
11. RViz frissül, látható a legközelebbi pont
```

---

## Gyakori hibák és megoldások

### 1. "No such file or directory: proba1_node"
**Ok:** Entry point nincs setup.py-ban vagy build nem történt meg
**Megoldás:**
```bash
colcon build --packages-select proba1
source install/setup.bash
```

### 2. "Could not find package 'proba1'"
**Ok:** Nem forrásoltad az overlay-t
**Megoldás:**
```bash
source install/setup.bash
```

### 3. RViz-ben nem látszik semmi
**Ok:** Fixed Frame rossz vagy topic név nem egyezik
**Megoldás:**
- Fixed Frame → `map` (vagy megfelelő frame a bag-ben)
- Topic nevek ellenőrzése: `ros2 topic list`

### 4. Bag lejátszás után node nem reagál
**Ok:** `use_sim_time` nincs beállítva vagy `--clock` hiányzik
**Megoldás:**
```xml
<param name="use_sim_time" value="true"/>
```
```bash
ros2 bag play /path/to/bag --clock
```

### 5. Import hibák Mac-en
**Ok:** Nincs ROS 2 telepítve
**Megoldás:** Normális, Ubuntu-n fog működni

---

## ZH stratégia

### Időbeosztás (90 perc)
1. **Package setup (10 perc):** setup.py, package.xml, könyvtárak
2. **Node váz (10 perc):** Importok, osztály, subscribers, publishers
3. **Odometry callback (10 perc):** Pozíció és yaw tárolás
4. **Path callback + legközelebbi pont (20 perc):** For loop, távolság számítás
5. **Szögszámítás (10 perc):** atan2, normalizálás
6. **Launch file (5 perc):** XML sablon másolás
7. **RViz konfig (10 perc):** Displays hozzáadása GUI-ban
8. **Build & teszt (15 perc):** colcon build, bag play, hibajavítás

### Prioritások
1. **Működő node** > Minden más
2. **Legközelebbi pont kiszámítása** > Szögszámítás
3. **Launch file** > RViz konfig
4. **Kommentek** > Tiszta kód

### Tippek
- Használd a repo-ban lévő példákat (gyak3, gyak6, gyak10, gyak12)!
- Copy-paste ÉS ÉRTS is!
- Tesztelj gyakran: `ros2 topic echo` a barátod
- Ha elakadsz: egyszerűsíts, működjön valami!

---

## Összefoglalás

### Mit tanultunk?
1. ✅ ROS 2 Python package struktúra
2. ✅ Subscriber és Publisher használat
3. ✅ Quaternion → Euler konverzió
4. ✅ Távolságszámítás, minimumkeresés
5. ✅ Szögszámítás 2D-ben (atan2)
6. ✅ Launch file XML
7. ✅ RViz konfiguráció
8. ✅ Bag file használat

### Kapcsolódó témák
- **TF transformations:** Koordináta-rendszerek közötti konverzió
- **Path planning:** Útvonalkövetés, Pure Pursuit
- **Sensor fusion:** Odometry + LaserScan kombinálása

### További gyakorlás
- Módosítsd: átlagold az N legközelebbi pontot
- Bővítsd: számold ki a távolságot is (Float64)
- Új feature: ha közel a legközelebbi pont, lassíts
- Vizualizáció: rajzold ki a vektort robot → legközelebbi pont (Marker)

---

**Sok sikert a ZH-n! 🚀**
