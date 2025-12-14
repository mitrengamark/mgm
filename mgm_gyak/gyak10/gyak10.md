# Gyak10 - Ackermann Kinematika és Pure Pursuit Path Követés

## Célkitűzés

Ez a gyakorlat a **robot mozgásvezérlés** alapjait tanítja meg, különös tekintettel az **Ackermann kormányzású robotok** kinematikájára és a **Pure Pursuit** path követési algoritmusra. A cél, hogy a robot autonóm módon kövessen egy előre definiált útvonalat.

### Mit tanulsz meg:
- **Ackermann kinematika**: hogyan mozog egy autó-szerű robot
- **Odometria**: pozíció és orientáció követése az idő függvényében
- **Pure Pursuit algoritmus**: path követés lookahead pont segítségével
- **Szabályozás**: sebesség és kormányszög kontroll
- **TF transzformáció**: robotpozíció lekérdezése a térben
- **Vizualizáció**: markerekkel jelölt kontrolltpontok RViz-ben

---

## A projekt szerkezete

```
gyak10/
├── gyak10/
│   ├── __init__.py
│   ├── ack_robot.py           # Ackermann robot szimulátor
│   └── pure_pursuit.py        # Pure Pursuit path követés
├── launch/
│   └── gyak10.launch.xml      # Launch fájl
├── rviz/
│   └── gyak10.rviz           # RViz konfiguráció
└── test/                      # Tesztfájlok
```

---

## Komponens 1: Ackermann Robot Szimulátor (`ack_robot.py`)

### Mi az az Ackermann kinematika?

Az Ackermann kormányzás az autók által használt klasszikus módszer. A robot egy hátsó tengely körül halad, az elülső kerék pedig fordítható (éppúgy, mint egy valódi autó).

#### Geometria

```
        +------------ front_wheel (kormányzható)
        |
     wheelbase
        |
        +------------ rear_axle (hajtás)
```

- **wheelbase**: hátsó és elülső tengely közötti távolság (~0.3 méter)
- **steering_angle (δ)**: az elülső kerék fordulási szöge
- **velocity (v)**: a hajtás sebessége

#### Kinematikai egyenletek

Ha a robot az origóban van, akkor egy `dt` idő alatt az új pozíció:

$$x_{new} = x + v \cdot \cos(\text{yaw}) \cdot dt$$

$$y_{new} = y + v \cdot \sin(\text{yaw}) \cdot dt$$

$$\text{yaw}_{new} = \text{yaw} + \frac{v}{\text{wheelbase}} \cdot \tan(\delta) \cdot dt$$

Az utolsó egyenlet azt írja le, hogy a robot orientációja (yaw) az elülső kerék fordulásával változik. Minél nagyobb a fordulási szög, annál jobban görbül az útvonal.

### A kódban

```python
class ACK_ROBOT(Node):
    def __init__(self):
        # Wheelbase paraméter: tényleges robot adataiból
        self.declare_parameter("wheel_base", 0.3)
        self.wheel_base = self.get_parameter("wheel_base").value

        # Robot állapot (x, y, yaw)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Parancsok fogadása: /cmd_vel topicról
        self.sub_cmd = self.create_subscription(Twist, "/cmd_vel", self.callback_cmd, 1)

        # Pozíció publikálása: odometria
        self.pub_odom = self.create_publisher(Odometry, "/odom", 1)

        # TF transzformáció: odom → base_link
        self.broadcaster = tf2_ros.TransformBroadcaster(self)

        # Statikus TF: base_link → front_wheel (nem változik)
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
```

#### Statikus transzformáció

A `base_link → front_wheel` transzformáció azt mutatja, hogy az elülső kerék a robot tengelytávnyi távolságra van előre:

```python
front_wheel = TransformStamped()
front_wheel.header.frame_id = "base_link"
front_wheel.child_frame_id = "front_wheel"
front_wheel.transform.translation.x = self.wheel_base  # 0.3 m előre
```

Ez **statikus**, mert az elülső kerék pozíciója a robothoz viszonyítva soha nem változik.

#### Timer callback: Ackermann kinematika alkalmazása

```python
def timer_callback(self):
    # Ackermann kinematikai egyenletek alkalmazása
    self.x += self.cmd.linear.x * self.dt * math.cos(self.yaw)
    self.y += self.cmd.linear.x * self.dt * math.sin(self.yaw)
    self.yaw += self.cmd.angular.z / self.wheel_base * self.cmd.linear.x * self.dt
```

Ez a 20 Hz-es szabályozási ciklusban:
1. Az új X és Y pozíciót az aktuális yaw szögből és sebességből számítja
2. Az yaw szöget a kormányzó parancsból (`cmd.angular.z`) frissíti

#### Odometry publikálása

Az odometria az elméleti pozíció alapján számított pozíció. Tartalmazza:
- `pose`: pozíció (X, Y) és orientáció (quaternion)
- `child_frame_id`: "base_link" (a robot test kerete)

A TF transzformáció ezt az információt továbbítja az RViz-nek és más ROS csomópontoknak.

---

## Komponens 2: Pure Pursuit Path Követés (`pure_pursuit.py`)

### Mi a Pure Pursuit?

A **Pure Pursuit** egy geometria-alapú path követési algoritmus. Az alapötlet egyszerű:
1. Megkeresed a legközelebbi pontot a path-on
2. Kiválasztasz egy **lookahead pontot** (a legközelebbi pont után, a lookahead_distance távolságban)
3. Kiszámítod a kormányszöget, amely a robot felé közelíti a lookahead pontot

#### Algoritmus lépésekben

```
Path:  o--o--o--o--o--o
       ^  ^     ^
       |  |     +-- Lookahead pont (távolabb, mint lookahead_distance)
       |  +-------- Legközelebbi pont
       +----------- Robot aktuális pozíciója
```

Az alapötlet: ha a robot a legközelebbi pont és a lookahead pont felé halad, akkor a path-ot követi.

### A kódban

#### Pozíció lekérdezése TF-ből

```python
# TF Buffer: robot pozíció lekérdezése
self.tfBuffer = tf2_ros.Buffer()
self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)

# Timer callback-ben: robot pozíció (map keretben)
if self.tfBuffer.can_transform("map", "base_link", ...):
    trans_base2map = self.tfBuffer.lookup_transform("map", "base_link", ...)
    self.pose_actual = do_transform_pose_stamped(base, trans_base2map)
```

Ez azt jelenti: **a robot valódi pozícióját a TF rendszerből lekérdezzük**, nem az odometriából. Ez rugalmasabb, mert ha más szenzorból (pl. SLAM) van jobb pozícióbecslés, azt használjuk.

#### Legközelebbi pont keresése

```python
index_closest_point = 0
closest_distance = 9999.0
for index in range(len(self.path.poses)):
    path_point = self.path.poses[index]
    dx_cal = path_point.pose.position.x - self.pose_actual.pose.position.x
    dy_cal = path_point.pose.position.y - self.pose_actual.pose.position.y
    distance_cal = math.sqrt(dx_cal**2 + dy_cal**2)
    if distance_cal < closest_distance:
        index_closest_point = index
        closest_distance = distance_cal
```

Ez a legközelebbi pont indexét és távolságát adja vissza.

#### Lookahead pont keresése

```python
lookahead_index = index_closest_point + 1
while lookahead_index < len(self.path.poses):
    lookahead_point = self.path.poses[lookahead_index]
    dx = lookahead_point.pose.position.x - self.pose_actual.pose.position.x
    dy = lookahead_point.pose.position.y - self.pose_actual.pose.position.y
    distance = math.sqrt(dx**2 + dy**2)
    if distance > self.lookahead_distance:  # Pl. 0.5 m
        break
    lookahead_index += 1
```

Ez az első pontot keresi meg, amely a lookahead_distance-nál távolabb van. Ha olyan pont nincsen (a path vége), az utolsó pontot használja.

#### Pure Pursuit kormányszög számítása

Az algoritmus magja:

$$\text{steering\_angle} = \arctan\left(\frac{2 \cdot \text{wheelbase} \cdot \sin(\alpha)}{L}\right)$$

ahol:
- $\alpha$ = szöghiba (target_yaw - current_yaw)
- $L$ = távolság a lookahead pontig

```python
# Szöghiba normalizálása [-π, π]-re
corrected_angle = math.atan2(math.sin(angular_error), math.cos(angular_error))

# Pure Pursuit kormányszög
steering_angle = math.atan(2 * self.wheelbase * math.sin(corrected_angle) / linear_error)

# Korlátozás: ne lépje túl a max_steering_speed-et
steering_angle = max(min(steering_angle, self.max_steering_speed), -self.max_steering_speed)
```

#### Sebesség szabályozás (Longitudinal Control)

```python
linear_output = 0.0
if linear_error > self.max_speed:
    linear_output = self.max_speed  # Korlát: maximum 0.1 m/s
elif linear_error > 0.1:
    linear_output = linear_error  # Egyébként: egyenes arányosság a távolsággal
```

Ez azt jelenti: ha messze van a lookahead pont, gyorsabban megy; ha közel van, lassabban.

#### Parancs publikálása

```python
cmd = Twist()
cmd.linear.x = linear_output      # Haladási sebesség
cmd.angular.z = steering_angle    # Kormányzó szög
self.pub_cmd.publish(cmd)
```

Ez a `cmd_vel` topicra megy, amit az `ack_robot` node fogad.

#### Vizualizáció: Markerek

```python
def send_point(self, closest_point: PoseStamped, ns: str):
    marker = Marker()
    marker.type = Marker.SPHERE  # Gömb alakú marker
    marker.pose = closest_point.pose
    marker.scale.x = 0.1  # 10 cm átmérő
    marker.color.r = 1.0  # Piros szín
    marker_array.markers.append(marker)
    self.pub_viz.publish(marker_array)
```

Ez olyan gömböket rajzol az RViz-be, amelyek a legközelebbi és lookahead pontokat mutatják. Ez nagyon hasznos a debugging során.

---

## Launch fájl (`gyak10.launch.xml`)

### Mit csinál?

```xml
<!-- 1. Statikus TF: map → odom -->
<node pkg="tf2_ros" exec="static_transform_publisher"
    args="-2.0 -0.5 0.0 0.0 0.0 0.0 map odom"/>
```

A robot a `(-2, -0.5)` pozícióból indul a map keretben.

```xml
<!-- 2. Path publikálása (gyak11 csomópontból) -->
<node pkg="gyak11" exec="pub_path" name="pub_path">
    <param name="text_file_name" value="...positions.txt"/>
</node>
```

Ez egy előre definiált útvonalat olvas és publikál `/path` topicon.

```xml
<!-- 3. Ackermann robot szimulátor -->
<node pkg="gyak10" exec="ack_robot" name="ack_robot">
    <param name="wheel_base" value="0.3"/>
</node>
```

A robotmotor: ezek az odometrikus pozícióadatok.

```xml
<!-- 4. Pure Pursuit kontroler -->
<node pkg="gyak10" exec="pure_pursuit" name="pure_pursuit">
    <param name="wheel_base" value="0.3"/>
    <param name="lookahead_distance" value="0.5"/>
    <param name="max_speed" value="0.1"/>
    <param name="max_steering_speed" value="0.6"/>
</node>
```

Ez a path követő csomópont. A paraméterek:
- **lookahead_distance**: mennyi messzire néz előre (0.5 m)
- **max_speed**: maximum 0.1 m/s
- **max_steering_speed**: maximum 0.6 rad/s kormányzás

```xml
<!-- 5. Path vizualizáció -->
<node pkg="gyak3" exec="path.py" name="path_node">
    <param name="path_topic_name" value="/path_odom"/>
    <param name="odom_topic_name" value="/odom"/>
</node>
```

Ez az odometria útvonalát követi (azaz ahol a robot volt).

```xml
<!-- 6. RViz vizualizáció -->
<node pkg="rviz2" exec="rviz2" args="-d $(find-pkg-share gyak10)/rviz/gyak10.rviz"/>
```

---

## RViz konfiguráció (`gyak10.rviz`)

### Megjelenítés

1. **Grid**: 10x10 méteres rács (a teret reprezentálja)
2. **TF**: koordinátarendszerek és transzformációk
   - `map`: világkoordináta-rendszer
   - `odom`: robot odometrikus kerete
3. **Path** (`/path`): zöld nyíl, az előírt útvonal
4. **Markers** (`/viz_closest`): piros gömbök
   - Legközelebbi pont: mutatja, hol van a robot a path-hoz képest
   - Lookahead pont: mutatja, hova megy a robot

**Fixed Frame**: `map` (abszolút referencia)

---

## Futtatás és működés

### Előfeltételések

```bash
# Workspace építése
cd ~/codes/mgm/mgm
colcon build --packages-select gyak10 gyak11 gyak3

# Beállítás
source install/setup.bash
```

### Indítás

```bash
# Launch fájl futtatása
ros2 launch gyak10 gyak10.launch.xml
```

Ez a következőket indítja egyszerre:
1. Statikus TF (`map → odom`)
2. Path publikálása
3. Ackermann robot szimulátort
4. Pure Pursuit kontrolert
5. Útvonal vizualizációt
6. RViz-et

### Mit látsz?

Az RViz-ben megjelennek:
- **Zöld nyíl** (Path): az előírt útvonal
- **Piros gömbök**: legközelebbi és lookahead pontok
- **Kék nyilak**: TF keretei

Az RViz-ben **Top-Down Ortho** nézetet használj a legjobb láthatóságért.

### Ellenőrzés

Külön terminálban:

```bash
# Topicok listája
ros2 topic list

# Robot sebesség parancsok
ros2 topic echo /cmd_vel --once

# Odometria adatok
ros2 topic echo /odom --once

# Vizualizációs markerek
ros2 topic echo /viz_closest --once
```

### Paraméterek finomhangolása

A launch fájlban módosíthatod:

```xml
<!-- Gyorsabb haladás -->
<param name="max_speed" value="0.2"/>

<!-- Agresszívabb kormányzás -->
<param name="max_steering_speed" value="1.0"/>

<!-- Kisebb lookahead (élesebb kanyar) -->
<param name="lookahead_distance" value="0.3"/>

<!-- Nagyobb lookahead (simább kanyar) -->
<param name="lookahead_distance" value="1.0"/>
```

---

## Főbb tanulságok

### 1. **Ackermann kinematika**
- Nem lehet irányváltás nélkül fordulni (mint egy autó)
- A wheelbase nagyobb, mint az irányváltási sugár

### 2. **Pure Pursuit algoritmus**
- Egyszerű, de hatékony path követésre
- A lookahead_distance megválasztása kritikus
  - Kis érték: agresszív, ingadozó viselkedés
  - Nagy érték: simább, de lassabb

### 3. **TF transzformáció**
- A robot valódi pozícióját a TF-ből lehet lekérdezni
- Ez rugalmasabb, mint az odometriára támaszkodás

### 4. **Szabályozás**
- Longitudinális (sebesség) és laterális (kormányzás) szabályozás külön
- Paraméterek finomhangolása a viselkedésért

### 5. **Vizualizáció**
- A markerek segítenek a debuggingban
- RViz-ben valós időben látod, mit csinál az algoritmus

---

## Gyakori hibák és megoldások

### "Cannot transform between frames"
- Ellenőrizd: `ros2 run tf2_tools view_frames.py`
- Szükséges: `map` → `odom` → `base_link` lánc

### Robot nem mozdul
- Ellenőrizd: `ros2 topic echo /cmd_vel`
- Van-e parancs az `ack_robot`-nak?

### Robot instabil (forogni kezd)
- Csökkentsd a `max_steering_speed`-et
- Vagy nagyobbra állítsd a `lookahead_distance`-et

### Path nem jelenik meg
- Ellenőrizd: `ros2 topic list | grep path`
- Szükséges: `pub_path` csomópont működik

### Markerek nem látszanak
- RViz: Add hozzá a `MarkerArray` display-t
- Topic: `/viz_closest`

---

## Továbbfejlesztési ötletek

1. **PID szabályozás**: PID helyett P szabályozást használ; PID javítaná
2. **Adaptív lookahead**: sebesség függő lookahead_distance
3. **Végpont**: ha a robot elég közel van az utolsó ponthoz, álljon meg
4. **Valós robototok**: Gazebo helyett valós TurtleBot3
5. **Szensorintegráció**: SLAM adatokat is felhasználni

---

## Hasznos parancsok

```bash
# Path fájl megtekintése
cat ~/codes/mgm/mgm/mgm_gyak/gyak11/param/positions.txt

# Launch fájl szerkesztése
gedit ~/codes/mgm/mgm/mgm_gyak/gyak10/launch/gyak10.launch.xml

# Csomópont logja
ros2 run gyak10 ack_robot --ros-args --log-level debug

# TF fa képként
ros2 run tf2_tools view_frames.py
dot -Tpng frames.gv -o frames.png

# Topic adatai (pl. path)
ros2 topic echo /path --once | head -50
```

---

## Referenciák

- [Pure Pursuit Control](https://en.wikipedia.org/wiki/Pursuit_curve)
- [Ackermann Steering Geometry](https://en.wikipedia.org/wiki/Ackermann_steering_geometry)
- [ROS2 TF2](https://docs.ros.org/en/humble/Concepts/Intermediate/Tf2/Tf2.html)
- [geometry_msgs/Twist](https://docs.ros2.org/latest/api/geometry_msgs/msg/Twist.html)
- [nav_msgs/Odometry](https://docs.ros2.org/latest/api/nav_msgs/msg/Odometry.html)

---

**Készült**: 2025. december  
**ROS verzió**: ROS 2 Humble  
**Tesztelve**: Pure Pursuit path követés szimulációban
