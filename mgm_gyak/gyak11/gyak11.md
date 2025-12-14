# Gyak11 - Path Publikálása és Mentése

## Célkitűzés

Ez a gyakorlat az **útvonalak (path-ok) kezelésének** alapjait tanítja meg ROS2-ben. A feladat az, hogy egy előre definiált útvonalat fájlból betöltünk és publikálunk, illetve fordítottan: a robot által bejárt útvonalat odometriából mentünk egy fájlba.

### Mit tanulsz meg:
- **Fájl olvasás és írás**: CSV formátumú adatok kezelése Python-ban
- **Path üzenet**: ROS2 `nav_msgs/Path` szerkezete és használata
- **PoseStamped**: egyedi pozíciók kezelése quaternion orientációval
- **Quaternion ↔ Euler**: szögtranszformációk (`yaw` szög és quaternion között)
- **Odometria feldolgozása**: robot pozíciójának követése az időben
- **TF transzformáció**: koordinátarendszerek közötti konverzió
- **Vizualizáció**: path-ok RViz-ben való megjelenítése
- **Ritkítás (Decimation)**: adatok szűrése - csak kellő távolság esetén mentés

---

## A projekt szerkezete

```
gyak11/
├── gyak11/
│   ├── __init__.py
│   ├── pub_path.py           # Útvonal publikálása fájlból
│   └── save_path.py          # Útvonal mentése odometriából
├── launch/
│   └── gyak11.launch.xml     # Launch fájl
├── param/
│   └── positions.txt         # CSV formátumú útvonal adatok
├── rviz/
│   └── gyak11.rviz          # RViz konfiguráció
├── ReadMe.md                 # Meglévő dokumentáció
└── test/                     # Tesztfájlok
```

---

## Adatformátum: `positions.txt`

### Szerkezet

Az útvonal adatok CSV formátumban tárolódnak:

```csv
x, y, z, yaw
-2.0, -0.5, 0.01, 0.0002
-1.9, -0.5, 0.01, 0.0002
-1.8, -0.5, 0.01, 0.0002
...
```

**Oszlopok:**
- **x**: pozíció az X tengelyen (méter)
- **y**: pozíció az Y tengelyen (méter)
- **z**: pozíció a Z tengelyen (általában 0 vagy szinte 0, 2D mozgáshoz)
- **yaw**: orientáció körül a Z tengely (fokokban, 0-360°)

**Jellemzők:**
- Minden sor egy pozt (időpillanatot) reprezentál
- Pozíciókat általában uniform intervallumokkal rögzítik (pl. minden 0.1 méterre)
- Az yaw szög a robot "nézési iránya"

---

## Komponens 1: Path Publikálása (`pub_path.py`)

### Célja

A `pub_path.py` egy **PathPublisher** csomópont, amely:
1. Beolvas egy CSV fájlt (`positions.txt`)
2. **Path üzenetté** konvertálja (ROS2 `nav_msgs/Path`)
3. **Periodikusan** publikálja a `/path` topicra (pl. 1 Hz-en)
4. **Vizualizációs markereket** hoz létre a path határpontjaihoz

### Szálkód lépésről lépésre

#### 1. Fájl beolvasása

```python
self.declare_parameter('text_file_name', '/workspace/src/.../positions.txt')
self.text_file_name = self.get_parameter('text_file_name').value

self.file_reader = open(self.text_file_name, 'r')
self.path = self.file_reader.readlines()  # Sorok listája
self.file_reader.close()
```

A fájlt sorokra (stringekre) olvassuk be.

#### 2. Adat feldolgozása

```python
self.path_values = []
for x in range(len(self.path)):
    for value in self.path[x].split(', '):  # Vesszővel elválasztott értékek
        self.path_values.append(float(value))

self.len_values = int(len(self.path_values) / 4)  # Hány póz van
```

- A `split(', ')` az egyes számokat szétválasztja
- Egy pózhoz 4 érték tartozik (x, y, z, yaw)

#### 3. Path üzenet létrehozása

```python
self.msg = Path()
self.msg.header.frame_id = "map"  # Vonatkozási keretrendszer

for i in range(self.len_values):
    # Yaw (fokból) radiánra, majd quaternionná
    yaw_rad = self.path_values[i*4+3] / 180.0 * math.pi
    q = quaternion_from_euler(0, 0, yaw_rad)  # (roll, pitch, yaw)
    
    # Egyedi PoseStamped üzenet
    msg_pose = PoseStamped()
    msg_pose.pose.position.x = self.path_values[i*4]
    msg_pose.pose.position.y = self.path_values[i*4+1]
    msg_pose.pose.position.z = self.path_values[i*4+2]
    msg_pose.pose.orientation.x = q[0]
    msg_pose.pose.orientation.y = q[1]
    msg_pose.pose.orientation.z = q[2]
    msg_pose.pose.orientation.w = q[3]
    
    # Hozzáadás a path-hoz
    self.msg.poses.append(msg_pose)
```

**Fontos:** A yaw szög (radiánban) **quaternion** formájúvá alakul, mert a ROS üzenetek ezt az orientációs formátumot használják.

#### 4. Quaternion és Euler szögek

**Mi a quaternion?**

A quaternion egy 4 értékű szám: `(x, y, z, w)`. Az orientáció tárolásának egy módja.

- **Euler szögek**: roll (X körül), pitch (Y körül), yaw (Z körül) - könnyen érthető, de gimbal lock probléma
- **Quaternion**: matematikailag stabil, a gimbal lock elkerülhető

A `quaternion_from_euler(roll, pitch, yaw)` átalakítás:
```
yaw = 45 fok = π/4 radián
→ quaternion = (0, 0, 0.383, 0.924)
```

Fordítva, az `euler_from_quaternion((x, y, z, w))` visszaalakítja.

#### 5. Bounding box és markerek

```python
# Szélsőértékek keresése
x_min, x_max = 9999, -9999
y_min, y_max = 9999, -9999
for pose in self.msg.poses:
    x, y = pose.pose.position.x, pose.pose.position.y
    x_min, x_max = min(x_min, x), max(x_max, x)
    y_min, y_max = min(y_min, y), max(y_max, y)

# Markerek: piros gömbök a sarok pontok
mark = Marker()
mark.type = Marker.SPHERE
mark.color.r = 1.0  # Piros
mark.scale.x = mark.scale.y = mark.scale.z = 0.1

# Marker 1: (x_min, y_min)
mark.id = 0
mark.pose.position.x, mark.pose.position.y = x_min, y_min
self.mark_array.markers.append(copy.deepcopy(mark))

# Marker 2: (x_max, y_max)
mark.id = 1
mark.pose.position.x, mark.pose.position.y = x_max, y_max
self.mark_array.markers.append(copy.deepcopy(mark))
```

Ez megmutatja, hogy az útvonal hol kezdődik és hol fejeződik be (körülbelül).

#### 6. Timer callback: publikálás

```python
def timer_callback(self):
    if self.frame_name == "map":
        # Egyszerű publikálás
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.msg)
    else:
        # Más keretrendszerbe transzformálás (TF-vel)
        if self.tfBuffer.can_transform("map", self.frame_name, ...):
            trans = self.tfBuffer.lookup_transform("map", self.frame_name, ...)
            for i in range(len(self.msg.poses)):
                self.msg.poses[i] = do_transform_pose_stamped(self.msg.poses[i], trans)
        self.publisher.publish(self.msg)
    
    # Markerek publikálása
    self.pub_viz.publish(self.mark_array)
```

Ez a 10 Hz-es (vagy beállított frekvenciájú) ciklusban publikálja az útvonalat.

---

## Komponens 2: Path Mentése (`save_path.py`)

### Célja

A `save_path.py` egy **PathSavingNode** csomópont, amely:
1. **Feliratkozik az odometria topicra** (pl. `/odom`)
2. **Rögzíti a robot pozícióját** az idő függvényében
3. **Fájlba írja** az adatokat CSV formátumban
4. **Ritkítést alkalmaz**: csak akkor ment, ha a távolság az előző mentetttől nagyobb, mint egy küszöb

### Szálkód lépésről lépésre

#### 1. Paraméterek és inicializálás

```python
self.declare_parameter("topic_name", "/odom")  # Odometria topic
self.declare_parameter("text_file_name", "/positions.txt")  # Fájl
self.declare_parameter("distance", 0.1)  # Minimális távolság mentések között (10 cm)

self.x, self.y, self.z = 0.0, 0.0, 0.0  # Előző mentett pozíció

# Fájl megnyitása írásra (figyelem: felülírja!)
self.file_writer = open(text_file_name, "w")

# Feliratkozás az odometriára
self.sub = self.create_subscription(Odometry, topic_name, self.odom_callback, 1)
```

#### 2. Odometria callback: ritkítás

```python
def odom_callback(self, msg: Odometry):
    # Távolság az előző mentett pozíciótól
    dx = self.x - msg.pose.pose.position.x
    dy = self.y - msg.pose.pose.position.y
    dz = self.z - msg.pose.pose.position.z
    distance = math.sqrt(dx**2 + dy**2 + dz**2)
    
    # Csak akkor ment, ha távolabb van, mint a küszöb (pl. 0.1 m)
    if distance > self.dist:
        # ... feldolgozás és mentés
        
        # Előző pozíció frissítése
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z
```

**Miért kell ritkítás?**

Az odometria általában 50 Hz-en (vagy még gyorsabban) jön. Ha minden adatot mentennénk, a fájl hatalmas lenne, és sok redundáns (nagyon közeli) pozíció lenne. Az 0.1 méteres küszöb azt jelenti, hogy csak akkor mentünk, ha a robot legalább 10 cm-t mozgott az utolsó mentés óta.

#### 3. TF transzformáció

```python
actual_pose = Pose()
if msg.header.frame_id == "map":
    # Már "map" keretben van
    actual_pose = msg.pose.pose
else:
    # Más keretről "map"-ra transzformálás
    if self.tfBuffer.can_transform("map", msg.header.frame_id, ...):
        trans = self.tfBuffer.lookup_transform("map", msg.header.frame_id, ...)
        actual_pose = do_transform_pose(msg.pose.pose, trans)
```

Az odometria általában az `odom` keretben jön, de a path-ot a `map` keretben akarjuk tárolni. A TF ezt megoldja.

#### 4. Quaternion → Yaw konverzió

```python
q = [
    actual_pose.orientation.x,
    actual_pose.orientation.y,
    actual_pose.orientation.z,
    actual_pose.orientation.w
]
yaw_radian = euler_from_quaternion(q)[2]  # Harmadik érték = yaw
yaw = yaw_radian / math.pi * 180.0  # Radiánból fokba
```

#### 5. Fájlba írás

```python
self.file_writer.write(str(round(actual_pose.position.x, 2)) + ', ')
self.file_writer.write(str(round(actual_pose.position.y, 2)) + ', ')
self.file_writer.write(str(round(actual_pose.position.z, 2)) + ', ')
self.file_writer.write(str(round(yaw, 4)))
self.file_writer.write('\n')
```

Ezt az egy-egy pózhoz az odometria callback alatt meghívva gyűjtjük az útvonalat a fájlba.

---

## Launch fájl (`gyak11.launch.xml`)

### Mit csinál?

```xml
<!-- Rosbag lejátszása (simuláció idővel) -->
<executable
    cmd="ros2 bag play --clock 1000 $(find-pkg-share gyak3)/bag"
    output="screen"
    shell="true"
/>
```

Ez egy rosbag fájlt játszik le (előre rögzített robot szenzoradatok). Ez a gazebo szimulációt helyettesíti.

```xml
<!-- Statikus TF: map → odom -->
<node pkg="tf2_ros" exec="static_transform_publisher"
    args="0.0 0.0 0.0 0.0 0.0 0.0 map odom"/>
```

Ez azt mondja, hogy az `odom` és `map` keretrendszer ugyanaz az origó.

```xml
<!-- Path PublishER (ENGEDÉLYEZVE) -->
<node pkg="gyak11" exec="pub_path" name="pub_path" output="screen">
    <param name="publish_rate" value="1.0"/>
    <param name="text_file_name" value="/workspace/.../positions.txt"/>
</node>
```

Ez az `pub_path` csomópontot indítja el, amely a `positions.txt` fájlból beolvasva publikálja az útvonalat.

```xml
<!-- Path SaveR (LETILTVA) -->
<!-- <node pkg="gyak11" exec="save_path" name="save_path" output="screen">
    <param name="distance" value="0.1"/>
    <param name="topic_name" value="/odom"/>
    <param name="text_file_name" value="...positions.txt"/>
</node> -->
```

Ez kommentálva van. Ha engedélyeznénk, a robot útvonalát mentené az odometriából.

```xml
<!-- RViz -->
<node pkg="rviz2" exec="rviz2" name="rviz2" args="-d $(find-pkg-share gyak11)/rviz/gyak11.rviz"/>
```

Az RViz megnyitása az előre konfigurált beállításokkal.

---

## RViz konfiguráció (`gyak11.rviz`)

### Megjelenítés

1. **Grid**: 10x10 méteres rács
2. **Odometry** (`/odom`): a robot pályája nyíl-alakban
3. **LaserScan** (`/scan`): a robot lézer szenzor által érzékelt környezet
4. **MarkerArray** (`/viz`): az útvonal határpontjai (piros gömbök)
5. **TF**: koordinátarendszerek

**Fixed Frame**: `map` (a világ abszolút kerete)

---

## Futtatás és működés

### Előfeltételések

```bash
# Workspace építése
cd ~/codes/mgm/mgm
colcon build --packages-select gyak11 gyak3

# Beállítás
source install/setup.bash
```

### Indítás

```bash
# Launch fájl futtatása
ros2 launch gyak11 gyak11.launch.xml
```

Ez a következőket indítja:
1. Rosbag lejátszást (szimulált szenzoradatok)
2. Path publikálást (`positions.txt` betöltésével)
3. RViz-et

### Mit látsz az RViz-ben?

- **Zöld nyilak**: az odometria által követett útvonal (robot valódi mozgása)
- **Piros gömbök**: az `positions.txt`-ben definiált útvonal határpontjai
- **Fehér pontok**: a lézer szenzor által érzékelt akadályok

Az ideális eset, hogy a **zöld nyilak** és a **piros gömbök** által jelölt útvonal körülbelül egybeesik.

### Ellenőrzés

Külön terminálban:

```bash
# Path topicjának ellenőrzése
ros2 topic echo /path --once | head -30

# Hány póz van?
ros2 topic echo /path --once | grep -c "position:"

# Vizualizációs markerek
ros2 topic echo /viz --once
```

---

## Zwei módok: Publikálás vs. Mentés

### Mód 1: Path Publikálása (Tanítás)

**Szcenario**: Van egy előre definiált útvonal (`positions.txt`), és azt szeretned, hogy a robot kövesse.

1. `pub_path` csomópont: betölti a fájlt és publikálja
2. `gyak10` (Pure Pursuit): feliratkozik az útvonalra és követi
3. Robot mozdul az útvonal szerint

### Mód 2: Path Mentése (Felvétel)

**Szcenario**: Manuálisan mozgatod a robotot (vagy egy demo futtatódik), és azt szeretnéd, hogy az útvonal mentésre kerüljön.

1. `save_path` csomópont: feliratkozik az odometriára és menti a fájlba
2. Robot mozdul (manuálisan vagy automata módon)
3. A végén a `positions.txt` frissül az új útvonallal

**Tipikus munkafolyamat:**
```
1. Felvétel: save_path engedélyezve → positions.txt feltöltödik
2. Indítás: pub_path engedélyezve, save_path letiltva → pub_path publikálja az új útvonalat
3. Követés: gyak10 indítása → robot követi az útvonalat
```

---

## Kvaterniók: Mélyebben

### Miért kellenek kvaterniók?

Az Euler szögek (roll, pitch, yaw) intuitívabbak, de problémája van: **gimbal lock**.

```
yaw = 90°
pitch = 90°
roll = ?  ← Ebben az esetben a roll és yaw azonos tengelyre vetül!
```

A kvaterniók nem szenvednek ebben a problémában. Ők egy matematikai "csoda" az 3D forgatásokhoz.

### Konverziók ROS2-ben

```python
# Yaw (fokból) quaternionra
yaw_rad = yaw_degree * math.pi / 180
q = quaternion_from_euler(0, 0, yaw_rad)  # (roll, pitch, yaw) → quaternion
# Eredmény: (x, y, z, w) tuple

# Quaternionból yaw
q = (x, y, z, w)
roll, pitch, yaw = euler_from_quaternion(q)
# yaw radiánban van!
```

---

## Gyakori hibák és megoldások

### "FileNotFoundError: ... positions.txt"
- Ellenőrizd a fájl útvonalát a launch fájlban
- Használj abszolút útvonalat, ne relatívat

### A path nem jelenik meg az RViz-ben
- Ellenőrizd: `ros2 topic list | grep path`
- Szükséges: a `pub_path` csomópont fut-e?
- RViz: Add hozzá a `Path` display-t és állítsd a topic-ot `/path`-ra

### A robot nem követi az útvonalat
- Ezt a `gyak10` csomagban kell debuggolni, nem itt
- De ellenőrizd: a path publikálódik-e? (`ros2 topic echo /path`)

### "Permission denied" fájlba íráskor
- Linux: a fájlnak írható hozzáférésre van szüksége
- `chmod 666 positions.txt`
- Vagy: másutt mentj a fájlt (home könyvtár)

### Quaternion konverziós hibák
- Győződj meg: a yaw radiánban van-e az `euler_from_quaternion` előtt
- Győződj meg: az `quaternion_from_euler` bemenete radiánban van-e

---

## Továbbfejlesztési ötletek

1. **Spline interpoláció**: egyes pontok közötti görbe
2. **Dinamikus frissítés**: path módosítása futás közben
3. **Több path**: több előre definiált útvonal közül választás
4. **Path simítás**: túl sok pont → kevesebb pontú görbe
5. **GPS integráció**: valódi GPS adatok nyomon követése

---

## Hasznos parancsok

```bash
# Fájl tartalma megtekintése
cat ~/codes/mgm/mgm/mgm_gyak/gyak11/param/positions.txt | head -20

# Fájl módosítása
gedit ~/codes/mgm/mgm/mgm_gyak/gyak11/param/positions.txt

# Python: fájl beolvasása és feldolgozása
python3 -c "
import csv
with open('positions.txt', 'r') as f:
    for line in f:
        x, y, z, yaw = map(float, line.split(', '))
        print(f'({x:.2f}, {y:.2f})')
"

# ROS: path hossza
ros2 topic echo /path --once | grep -c "pose:"

# CSV statisztika
python3 -c "
import numpy as np
data = np.loadtxt('positions.txt', delimiter=',')
print(f'Pontok száma: {len(data)}')
print(f'X tartomány: {data[:,0].min():.2f} - {data[:,0].max():.2f}')
print(f'Y tartomány: {data[:,1].min():.2f} - {data[:,1].max():.2f}')
"
```

---

## Referenciák

- [ROS2 nav_msgs/Path](https://docs.ros2.org/latest/api/nav_msgs/msg/Path.html)
- [ROS2 geometry_msgs/PoseStamped](https://docs.ros2.org/latest/api/geometry_msgs/msg/PoseStamped.html)
- [TF Transformations (quaternion, Euler)](https://github.com/ros/geometry/tree/humble-devel/tf_transformations)
- [Quaternion](https://en.wikipedia.org/wiki/Quaternion)
- [ROS2 TF2](https://docs.ros.org/en/humble/Concepts/Intermediate/Tf2/Tf2.html)

---

**Készült**: 2025. december  
**ROS verzió**: ROS 2 Humble  
**Tesztelve**: Path publikálása és mentése szimulációban
