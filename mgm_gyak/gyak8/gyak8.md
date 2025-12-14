# gyak8 – TF Transzformációk használata és statikus frame-ek

A `gyak8` csomag a ROS 2 **TF (Transform)** rendszer használatát mutatja be részletesen. A node **lekérdezi** a TF transzformációkat különböző robot komponensek (kerekek) pozíciójának meghatározásához, és **publikál** egy **statikus TF-et** (`safety_left` frame). Az eredmények vizualizálásához több path-t is megjelenít RViz-ben.

---

## 🎯 Mi a cél?

- **TF Buffer és Listener**: transzformációk fogadása és tárolása.
- **TF lekérdezés**: `lookup_transform()` használata (frame-ek közötti pozíció).
- **Statikus TF publikálás**: `StaticTransformBroadcaster` használata.
- **can_transform()**: ellenőrzés, hogy létezik-e transzformáció.
- **Több path vizualizáció**: robot középvonala, kerekek, és safety frame nyomkövetése.
- **Koordináta-rendszerek közötti számítások**: gyakorlati alkalmazás.

---

## 📁 Fájlstruktúra

```
mgm_gyak/gyak8/
├── package.xml                   # Csomag metaadatok (ament_python)
├── setup.py                      # Python entry points (test_tf)
├── setup.cfg                     # Python setuptools konfig
├── gyak8/
│   ├── __init__.py
│   └── test_tf.py               # WheelPathHandler node: TF lekérdezés és publikálás
├── launch/
│   └── gyak8.launch.xml         # Rosbag + test_tf + 4× path node + RViz
├── rviz/
│   └── gyak8.rviz               # RViz konfig (4 path, TF, LaserScan, RobotModel)
├── resource/
│   └── gyak8
└── test/                        # Tesztek
```

Főbb fájlok:
- [gyak8/test_tf.py](gyak8/test_tf.py) – TF lekérdezés és statikus TF publikálás
- [launch/gyak8.launch.xml](launch/gyak8.launch.xml) – komplex launch fájl
- [rviz/gyak8.rviz](rviz/gyak8.rviz) – 4 path megjelenítése

---

## 🔧 A WheelPathHandler node működése

### Áttekintés

[gyak8/test_tf.py](gyak8/test_tf.py)

A `WheelPathHandler` node:
1. **Létrehoz egy statikus TF-et**: `base_link → safety_left` (0.5 m balra).
2. **Lekérdezi a TF-eket**: `odom → wheel_left_link`, `odom → wheel_right_link`, `odom → safety_left`.
3. **Publikálja az odometriákat**: `/odom_left`, `/odom_right`, `/odom_safety_left`.
4. Ezekből **path-ok épülnek** (gyak3 `path.py` node-okkal).

### Mi az a TF (Transform)?

**TF = Transform Framework**

A TF egy ROS rendszer a **koordinátarendszerek (frame-ek) közötti kapcsolatok** kezelésére. Minden robot résznek van saját koordinátarendszere (frame), és a TF tárolja ezek közötti transzformációkat (pozíció + orientáció).

**Példa koordinátarendszerek:**
```
odom (globális)
  └─ base_footprint
      └─ base_link (robot teste)
          ├─ wheel_left_link (bal kerék)
          ├─ wheel_right_link (jobb kerék)
          ├─ base_scan (LiDAR)
          └─ safety_left (biztonsági pont, egyedi)
```

**Miért fontos?**
- Különböző szenzorok különböző koordinátarendszerekben mérnek.
- A TF automatikusan átszámítja a pozíciókat frame-ek között.
- Nem kell manuálisan transzformációs mátrixokat számolni.

---

### Részletes kód magyarázat

#### Inicializálás – TF Buffer és Listener

```python
class WheelPathHandler(Node):
    def __init__(self):
        super().__init__('test_viz')
        
        self.declare_parameter('target_frame', 'odom')
        self.target_frame = self.get_parameter('target_frame').value
        
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self)
```

**TF Buffer:**
- Tárolja az összes transzformációt.
- Időbélyegekkel együtt (múltbeli transzformációk is lekérdezhetők).
- Automatikusan interpolál időpontok között.

**TF Listener:**
- Feliratkozik a `/tf` és `/tf_static` topicokra.
- Minden beérkező transzformációt eltárol a Buffer-ben.
- Háttérben fut, automatikusan frissít.

**Target frame:**
- A cél koordinátarendszer, amelyben a pozíciókat szeretnénk (alapból: `odom`).

---

#### Statikus TF publikálás

```python
self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
safety_transform = tf2_ros.TransformStamped()

safety_transform.header.stamp = self.get_clock().now().to_msg()
safety_transform.header.frame_id = "base_link"  # Szülő
safety_transform.child_frame_id = "safety_left"  # Gyerek

# Transzláció: 0.5 m balra (y irány)
safety_transform.transform.translation.x = 0.0
safety_transform.transform.translation.y = 0.5
safety_transform.transform.translation.z = 0.0

# Rotáció: nincs forgatás
safety_transform.transform.rotation.x = 0.0
safety_transform.transform.rotation.y = 0.0
safety_transform.transform.rotation.z = 0.0
safety_transform.transform.rotation.w = 1.0

self.static_broadcaster.sendTransform(safety_transform)
```

**Mi az a statikus TF?**
- **Statikus**: nem változik időben (állandó kapcsolat).
- **Dinamikus TF** (pl. `odom → base_link`): változik, ahogy a robot mozog.

**Példa:**
- `base_link → wheel_left_link`: statikus (kerék mindig ugyanott van a robothoz képest).
- `odom → base_link`: dinamikus (robot mozog a világban).

**Miért `safety_left`?**
- Egy biztonsági pont a robot bal oldalán (pl. ütközéselkerüléshez).
- 0.5 m-re balra a robot közepétől.
- A TF rendszerben most már `odom → base_link → safety_left` láncban elérhető.

**StaticTransformBroadcaster vs. TransformBroadcaster:**
- **Static**: egyszer publikálva, örökre érvényes.
- **Dynamic**: folyamatosan újra kell publikálni (pl. timer-rel).

---

#### Publisher-ek létrehozása

```python
self.pub_left = self.create_publisher(Odometry, '/odom_left', 1)
self.pub_right = self.create_publisher(Odometry, '/odom_right', 1)
self.pub_safety_left = self.create_publisher(Odometry, '/odom_safety_left', 1)

self.timer = self.create_timer(0.1, self.timer_callback)
```

**3 odometria publisher:**
- `/odom_left`: bal kerék pozíciója az `odom` frame-ben.
- `/odom_right`: jobb kerék pozíciója az `odom` frame-ben.
- `/odom_safety_left`: safety_left frame pozíciója az `odom` frame-ben.

**Timer:** 10 Hz (0.1 sec) – rendszeresen lekérdezi a TF-eket.

---

#### Timer callback – TF lekérdezések

```python
def timer_callback(self):
    self.publish_side_position("wheel_left_link", self.pub_left)
    self.publish_side_position("wheel_right_link", self.pub_right)
    self.publish_side_position("safety_left", self.pub_safety_left)
```

**Hívja a `publish_side_position()` függvényt** 3× különböző frame-ekhez.

---

#### TF lekérdezés és publikálás

```python
def publish_side_position(self, wheel_frame: str, publisher):
    now = rclpy.time.Time()
```

**`now = rclpy.time.Time()`:**
- "Most" időpont (legfrissebb elérhető transzformáció).
- Alternatíva: konkrét időbélyeg (múltbeli transzformáció lekérdezése).

---

##### can_transform() – ellenőrzés

```python
if self.tfBuffer.can_transform(self.target_frame, wheel_frame, 
                                time=now, 
                                timeout=rclpy.duration.Duration(seconds=0.5)):
```

**Miért kell ellenőrizni?**
- A TF transzformáció nem biztos, hogy azonnal elérhető.
- Ha a frame még nem létezik a TF fa-ban → `lookup_transform()` hibát dobna.

**Paraméterek:**
- `self.target_frame`: "odom" (ahonnan)
- `wheel_frame`: pl. "wheel_left_link" (ahova)
- `time=now`: aktuális időpont
- `timeout=0.5 sec`: várjon max 0.5 másodpercet

**Visszatérés:**
- `True`: a transzformáció elérhető.
- `False`: nem elérhető (pl. frame még nem publikálva).

---

##### lookup_transform() – lekérdezés

```python
trans = self.tfBuffer.lookup_transform(self.target_frame, wheel_frame, now)
```

**Mit csinál?**
- Lekérdezi a transzformációt `target_frame → wheel_frame`.
- Visszaad egy `TransformStamped` üzenetet.

**TransformStamped struktúra:**
```python
header:
  stamp: ...
  frame_id: "odom"
child_frame_id: "wheel_left_link"
transform:
  translation:
    x: 1.5    # A wheel_left_link x pozíciója az odom frame-ben
    y: 0.2    # Y pozíció
    z: 0.0    # Z pozíció
  rotation:   # Quaternion orientáció
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
```

**Példa értelmezés:**
- "A bal kerék (`wheel_left_link`) az `odom` frame-ben az (1.5, 0.2, 0.0) pozícióban van."

---

##### Odometria üzenet létrehozása

```python
odom = Odometry()
odom.header.stamp = trans.header.stamp
odom.header.frame_id = self.target_frame
odom.child_frame_id = trans.child_frame_id
odom.pose.pose.position.x = trans.transform.translation.x
odom.pose.pose.position.y = trans.transform.translation.y
odom.pose.pose.position.z = trans.transform.translation.z
odom.pose.pose.orientation = trans.transform.rotation

publisher.publish(odom)
```

**Miért Odometry?**
- Az `Odometry` üzenet tartalmazza a pozíciót és orientációt.
- A `gyak3/path.py` node `Odometry` üzeneteket vár bemenetként.
- Így a TF transzformációból építhetünk path-okat.

---

## 🚀 Launch fájl – komplex architektúra

[launch/gyak8.launch.xml](launch/gyak8.launch.xml)

```xml
<launch>
    <!-- Rosbag play -->
    <executable cmd="ros2 bag play --clock 1000 $(find-pkg-share gyak3)/bag" .../>

    <!-- TF handler node -->
    <node pkg="gyak8" exec="test_tf" name="test_tf" output="screen">
        <param name="use_sim_time" value="true"/>
        <param name="target_frame" value="odom"/>
    </node>

    <!-- Path: robot közepe (eredeti odometria) -->
    <node pkg="gyak3" exec="path.py" name="path_mid_node" ...>
        <param name="path_topic_name" value="/path_mid"/>
        <param name="odom_topic_name" value="/odom"/>
    </node>

    <!-- Path: bal kerék -->
    <node pkg="gyak3" exec="path.py" name="path_left_node" ...>
        <param name="path_topic_name" value="/path_left"/>
        <param name="odom_topic_name" value="/odom_left"/>
    </node>

    <!-- Path: jobb kerék -->
    <node pkg="gyak3" exec="path.py" name="path_right_node" ...>
        <param name="path_topic_name" value="/path_right"/>
        <param name="odom_topic_name" value="/odom_right"/>
    </node>

    <!-- Path: safety_left frame -->
    <node pkg="gyak3" exec="path.py" name="path_safety_left_node" ...>
        <param name="path_topic_name" value="/path_safety_left"/>
        <param name="odom_topic_name" value="/odom_safety_left"/>
    </node>

    <!-- RViz -->
    <node pkg="rviz2" exec="rviz2" .../>
</launch>
```

**Architektúra:**
```
Rosbag → /odom, /scan, /tf
    ↓
WheelPathHandler (test_tf)
    ├─ Létrehoz: safety_left static TF
    ├─ Lekérdez: odom → wheel_left_link
    ├─ Lekérdez: odom → wheel_right_link
    └─ Lekérdez: odom → safety_left
    ↓
Publikál: /odom_left, /odom_right, /odom_safety_left
    ↓
4× PathCreator node (gyak3)
    ├─ /path_mid ← /odom
    ├─ /path_left ← /odom_left
    ├─ /path_right ← /odom_right
    └─ /path_safety_left ← /odom_safety_left
    ↓
RViz: 4 különböző színű path
```

**Eredmény:**
- **Zöld vonal**: robot középvonala (eredeti `/odom`).
- **Kék vonal**: bal kerék nyoma.
- **Rózsaszín vonal**: jobb kerék nyoma.
- **Zöld vonal (vastagabb)**: safety_left frame nyoma (0.5 m balra).

---

## 🖼️ RViz konfiguráció

[rviz/gyak8.rviz](rviz/gyak8.rviz)

- Fix Frame: `odom`
- Megjelenítők:
  - `Grid` – referencia rács
  - `Odometry` – `/odom` (robot pozíció)
  - `LaserScan` – `/scan` (LiDAR adatok)
  - `RobotModel` – robot vizualizáció (URDF-ből)
  - **`TF`** – koordinátarendszer fa megjelenítése
  - **`PathMid`** – `/path_mid` (zöld, robot közepe)
  - **`PathLeft`** – `/path_left` (kék, bal kerék)
  - **`PathRight`** – `/path_right` (rózsaszín, jobb kerék)
  - **`PathSafetyLeft`** – `/path_safety_left` (zöld, safety frame)

**TF Display:**
- Mutatja az összes frame-et és kapcsolatukat.
- Látható a `safety_left` frame is a `base_link` alatt.

---

## 📦 Build és futtatás

```bash
# Build (gyak3 is kell a path.py miatt!)
colcon build --packages-select gyak3 gyak8

# Forrásold a környezetet
source install/setup.bash

# Indítás
ros2 launch gyak8 gyak8.launch.xml
```

**Mit látsz RViz-ben?**
- 4 párhuzamos path vonal:
  - Középen: robot középvonala (zöld).
  - Kicsit balra: bal kerék (kék).
  - Kicsit jobbra: jobb kerék (rózsaszín).
  - Még balrább: safety_left (zöld, vastagabb).
- TF megjelenítő: láthatók a frame-ek (színes tengelyek).

**Ellenőrzés:**
```bash
# Topic-ok
ros2 topic list
ros2 topic echo /odom_left
ros2 topic echo /odom_safety_left

# TF ellenőrzés
ros2 run tf2_tools view_frames
# (generál egy frames.pdf-et a TF fa vizualizációval)

# TF echo: odom → safety_left transzformáció
ros2 run tf2_ros tf2_echo odom safety_left

# Statikus TF-ek listája
ros2 topic echo /tf_static
```

---

## 🎓 Mit tanulsz ebből?

### 1. TF Buffer és Listener
- `tf2_ros.Buffer()`: transzformációk tárolása.
- `tf2_ros.TransformListener()`: transzformációk fogadása.
- Automatikus frame interpoláció és időbélyeg kezelés.

### 2. TF lekérdezés
- `can_transform()`: ellenőrzés timeout-tal.
- `lookup_transform()`: transzformáció lekérdezése.
- `TransformStamped` üzenet struktúra.

### 3. Statikus vs. dinamikus TF
- **Statikus**: `StaticTransformBroadcaster`, egyszer publikálva.
- **Dinamikus**: `TransformBroadcaster`, folyamatos frissítés.

### 4. Koordinátarendszer láncok
- Frame hierarchia: `odom → base_link → wheel_left_link`.
- Tranzitív lekérdezés: `odom → wheel_left_link` automatikusan számított.

### 5. TF → Odometry konverzió
- TF transzformációból odometria építése.
- Újrahasználható path építéshez.

---

## 🔍 Gyakori hibák

| Probléma | Ok | Megoldás |
|---|---|---|
| `ExtrapolationException` | Transzformáció nem elérhető az adott időpontban | Növeld a `timeout`-ot vagy használj `time=now` |
| `LookupException` | A frame nem létezik a TF fa-ban | Ellenőrizd: `ros2 run tf2_tools view_frames` |
| Nincs path | A path node nem kap odometriát | Ellenőrizd: `ros2 topic echo /odom_left` |
| `safety_left` nem látszik | Statikus TF nem publikálódott | Ellenőrizd: `ros2 topic echo /tf_static` |
| Rossz koordináták | Rossz target_frame | Állítsd `target_frame="odom"`-ra |

---

## 💡 Gyakorlási ötletek

1. **Adj hozzá több statikus TF-et:**
   ```python
   safety_right = ...
   safety_right.transform.translation.y = -0.5  # Jobbra
   ```

2. **Változtasd a target_frame-et:**
   - Állítsd `base_link`-re → path-ok a robot helyi koordinátarendszerében.

3. **Dinamikus TF publikálás:**
   - Használj `TransformBroadcaster` helyett `StaticTransformBroadcaster`.
   - Timer-ben folyamatosan frissítsd.

4. **TF transzformációk chain-ben:**
   - Kérdezd le: `base_link → wheel_left_link`.
   - Számítsd ki a távolságot a két kerék között.

5. **Integráld a LaserScan-nel:**
   - Transzformáld a scan pontokat `odom` frame-be.
   - Lásd: `gyak6` + TF kombinálása.

---

## 📚 Hasznos ROS2 parancsok

```bash
# TF fa vizualizáció (PDF generálás)
ros2 run tf2_tools view_frames

# TF echo (valós idejű transzformáció)
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo odom safety_left

# Statikus TF-ek megtekintése
ros2 topic echo /tf_static

# Dinamikus TF-ek megtekintése
ros2 topic echo /tf

# Frame lista (RViz TF display-ből is látszik)
ros2 run tf2_ros tf2_monitor

# TF késleltetések ellenőrzése
ros2 run tf2_ros tf2_monitor odom base_link
```

---

## 🎯 Vizsgára felkészülés

### Amit tudnod kell:
- ✅ Mi az a TF (Transform Framework)?
- ✅ TF Buffer és Listener szerepe.
- ✅ `lookup_transform()` használata.
- ✅ `can_transform()` ellenőrzés timeout-tal.
- ✅ Statikus vs. dinamikus TF különbsége.
- ✅ `StaticTransformBroadcaster` használata.
- ✅ Frame hierarchia (parent → child).
- ✅ `TransformStamped` üzenet struktúra.

### Vizsgán gyakori feladatok:
- TF lekérdezés implementálása.
- Statikus TF publikálás.
- Frame-ek közötti távolság számítása.
- Koordináta-transzformáció (pont átalakítása egyik frame-ből másikba).
- TF fa hibakeresés (hiányzó frame, késleltetés).

---

## 🏁 Összefoglalás

A `gyak8` csomag a ROS 2 TF rendszer gyakorlati használatát mutatja be. A node statikus TF-et publikál (`safety_left`), lekérdezi különböző frame-ek pozícióját a TF rendszerből, és ezekből odometria üzeneteket épít. A többszörös path vizualizáció szemléletesen mutatja, hogyan követheted nyomon különböző robot komponensek mozgását. Ez a tudás elengedhetetlen komplex robotikai alkalmazásokhoz, ahol több szenzor és aktor koordinátarendszereit kell kezelni.

**Sok sikert a tanuláshoz és a vizsgához!** 🚀
