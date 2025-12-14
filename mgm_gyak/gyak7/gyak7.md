# gyak7 – PID Szabályozó és Differenciális Robot Szimuláció

A `gyak7` csomag egy komplett robot vezérlési rendszert valósít meg: egy **differenciális robot szimulátort** (kinematikai modell) és egy **PID szabályozót** a póz követéshez. Ez a csomag kiválóan demonstrálja a feedback szabályozás és a robot kinematika alapjait.

---

## 🎯 Mi a cél?

- **Differenciális robot szimuláció**: egyszerű kinematikai modell megvalósítása.
- **PID szabályozó**: Proportional-Integral-Derivative vezérlés tanulása.
- **Póz követés**: a robot eljut egy célpozícióba és célirányba is beáll.
- **TF kezelés**: koordinátarendszer transzformációk (odom → base_link).
- **Valós idejű szabályozás**: timer alapú eseményvezérlés.
- **Interaktív célmegadás**: RViz-ben "2D Goal Pose" eszközzel kézzel beállítható a cél.

---

## 📁 Fájlstruktúra

```
mgm_gyak/gyak7/
├── package.xml                   # Csomag metaadatok (ament_python)
├── setup.py                      # Python entry points (diff_robot, control)
├── setup.cfg                     # Python setuptools konfig
├── gyak7/
│   ├── __init__.py
│   ├── diff_robot.py            # DIFF_ROBOT node: robot szimulátor
│   └── control.py               # PID_CONTROL node: PID szabályozó
├── launch/
│   └── gyak7.launch.xml         # Mindkét node + RViz indítása
├── rviz/
│   └── gyak7.rviz               # RViz konfig (Odometry, TF, Goal Pose)
├── resource/
│   └── gyak7
└── test/                        # Tesztek
```

Főbb fájlok:
- [gyak7/diff_robot.py](gyak7/diff_robot.py) – differenciális robot szimulátor
- [gyak7/control.py](gyak7/control.py) – PID szabályozó
- [launch/gyak7.launch.xml](launch/gyak7.launch.xml) – indító fájl
- [rviz/gyak7.rviz](rviz/gyak7.rviz) – RViz beállítások

---

## 🤖 DIFF_ROBOT node – Robot szimulátor

### Áttekintés

[gyak7/diff_robot.py](gyak7/diff_robot.py)

A `DIFF_ROBOT` node egy egyszerű **differenciális robot kinematikai modellt** valósít meg. Feliratkozik a `/cmd_vel` topicra (sebesség parancsok), frissíti a robot pozícióját (x, y, yaw), és publikálja az odometriát (`/odom`) valamint a TF transzformációt (`odom → base_link`).

### Mi az a differenciális robot?

A **differenciális robot** két független kerékkel rendelkezik (bal és jobb), amelyek különböző sebességekkel foroghatnak. Így képes:
- **Előre/hátra mozogni** (mindkét kerék azonos sebességgel).
- **Forogni** (kerekek ellentétes irányban).
- **Ívben haladni** (kerekek különböző sebességgel).

**Egyszerűsített modell** (ami itt van):
- Bemenet: `linear.x` (előre/hátra sebesség, m/s) és `angular.z` (szögsebesség, rad/s).
- Nincs keréksebesség számítás – közvetlenül a test sebességekkel dolgozunk.

### Kinematikai egyenletek

```python
x += v * dt * cos(yaw)
y += v * dt * sin(yaw)
yaw += omega * dt
```

**Ahol:**
- `x, y`: robot pozíciója a síkon (m)
- `yaw`: robot orientációja/irány (rad)
- `v`: lineáris sebesség (`cmd.linear.x`)
- `omega`: szögsebesség (`cmd.angular.z`)
- `dt`: mintavételi idő (0.05 sec = 20 Hz)

**Matematikai magyarázat:**
- A robot mindig az aktuális `yaw` irányába mozog.
- `cos(yaw)` és `sin(yaw)` vetítik a sebességet x és y irányra.
- A szögsebesség közvetlenül változtatja az orientációt.

### Részletes kód magyarázat

#### Inicializálás

```python
class DIFF_ROBOT(Node):
    def __init__(self):
        super().__init__('diff_robot')
        
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        
        self.cmd = Twist()
        
        self.broadcaster = tf2_ros.TransformBroadcaster(self)
        self.pub_odom = self.create_publisher(Odometry, "/odom", 1)
        self.sub_cmd = self.create_subscription(Twist, "/cmd_vel", self.callback_cmd, 1)
        
        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.timer_callback)
```

**Állapotváltozók:**
- `x, y, yaw`: robot pozíció és orientáció (kezdetben origó).
- `cmd`: utoljára kapott sebesség parancs.

**Kommunikáció:**
- Subscriber: `/cmd_vel` (Twist) – sebességparancsok fogadása.
- Publisher: `/odom` (Odometry) – odometria publikálása.
- TF Broadcaster: `odom → base_link` transzformáció.

**Timer:** 20 Hz (0.05 sec) – nagy frissítési frekvencia a sima mozgáshoz.

---

#### Sebesség parancs fogadása

```python
def callback_cmd(self, msg: Twist):
    self.cmd = msg
```

**Egyszerű tárolás**: a legutóbbi parancsot eltároljuk, és a timer callback-ben használjuk.

---

#### Timer callback – pozíció frissítése

```python
def timer_callback(self):
    # Kinematika frissítése
    self.x += self.cmd.linear.x * self.dt * math.cos(self.yaw)
    self.y += self.cmd.linear.x * self.dt * math.sin(self.yaw)
    self.yaw += self.cmd.angular.z * self.dt
    
    # Odometry üzenet
    odom = Odometry()
    odom.header.stamp = self.get_clock().now().to_msg()
    odom.header.frame_id = "odom"
    odom.child_frame_id = "base_link"
    odom.pose.pose.position.x = self.x
    odom.pose.pose.position.y = self.y
```

**Fontos részletek:**
- `frame_id = "odom"`: globális koordinátarendszer (világhoz kötött).
- `child_frame_id = "base_link"`: robot helyi koordinátarendszere.

---

#### Quaternion konverzió

```python
from tf_transformations import quaternion_from_euler

q = quaternion_from_euler(0, 0, self.yaw)  # (roll, pitch, yaw)
odom.pose.pose.orientation.x = q[0]
odom.pose.pose.orientation.y = q[1]
odom.pose.pose.orientation.z = q[2]
odom.pose.pose.orientation.w = q[3]
```

**Miért quaternion?**
- ROS orientációkat quaternion-nal reprezentál (matematikailag stabil).
- **Euler-szögek** (roll, pitch, yaw): ember számára érthető (fok/radián).
- **Quaternion**: 4 érték (x, y, z, w) – gimbal lock nélküli reprezentáció.

**2D esetben:**
- `roll = 0`: nincs dőlés oldalra.
- `pitch = 0`: nincs dőlés előre/hátra.
- `yaw = θ`: forgatás Z tengely körül (a robot "nézési iránya").

---

#### TF publikálás

```python
tf_stamped = TransformStamped()
tf_stamped.header = odom.header
tf_stamped.child_frame_id = odom.child_frame_id

tf_stamped.transform.translation.x = odom.pose.pose.position.x
tf_stamped.transform.translation.y = odom.pose.pose.position.y
tf_stamped.transform.translation.z = odom.pose.pose.position.z

tf_stamped.transform.rotation = odom.pose.pose.orientation

self.broadcaster.sendTransform(tf_stamped)
```

**Mi az a TF (Transform)?**
- Koordinátarendszerek közötti kapcsolat (transzláció + rotáció).
- `odom → base_link`: "hol van a robot a világban?".
- RViz-ben így jelenik meg a robot helyes pozícióban.

---

## 🎛️ PID_CONTROL node – Szabályozó

### Áttekintés

[gyak7/control.py](gyak7/control.py)

A `PID_CONTROL` node egy **PID szabályozót** valósít meg, amely a robotot egy célpozícióba (`/goal_pose`) irányítja. Feliratkozik az odometriára (`/odom`) és a célpozícióra, kiszámítja a szükséges sebességeket (longitudinális és laterális), és publikálja a `/cmd_vel` topicra.

### Mi az a PID szabályozó?

**PID = Proportional-Integral-Derivative (Arányos-Integráló-Deriváló)**

Ez egy feedback szabályozó, amely a **hiba** (különbség a kívánt és aktuális érték között) alapján számítja ki a **beavatkozást** (sebesség parancs).

```
u(t) = Kp·e(t) + Ki·∫e(t)dt + Kd·de(t)/dt
```

**Tagok:**
- **P (Proportional)**: hibával arányos beavatkozás.
  - Nagy hiba → nagy beavatkozás.
  - `Kp = 10.0`: agresszív reakció.
  
- **I (Integral)**: felhalmozott hiba integrálja.
  - Kiküszöböli a maradó hibát (steady-state error).
  - `Ki = 0.0`: itt nincs használva (egyszerűsítés).

- **D (Derivative)**: hiba változási sebessége.
  - Csökkenti a túllendülést és stabilizálja.
  - `Kd = 0.0`: itt nincs használva (egyszerűsítés).

**Ebben a példában: csak P szabályozó van (`Kp = 10.0`), I és D = 0.**

---

### Részletes kód magyarázat

#### Inicializálás

```python
class PID_CONTROL(Node):
    def __init__(self):
        super().__init__('control_node')
        
        self.declare_parameter('Kp', 1.0)
        self.declare_parameter('Ki', 0.0)
        self.declare_parameter('Kd', 0.0)
        self.Kp = self.get_parameter('Kp').value
        self.Ki = self.get_parameter('Ki').value
        self.Kd = self.get_parameter('Kd').value
        
        self.declare_parameter('max_speed', 0.1)
        self.max_speed = self.get_parameter('max_speed').value
        self.declare_parameter('max_angular_speed', 0.3)
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
```

**Paraméterek:**
- `Kp, Ki, Kd`: PID együtthatók (launch fájlból beállíthatók).
- `max_speed`: maximális lineáris sebesség (m/s).
- `max_angular_speed`: maximális szögsebesség (rad/s).

**PID állapotváltozók:**
```python
self.integral = 0.0
self.previous_error = 0.0
self.dt = 0.1  # 10 Hz szabályozási ciklus
```

---

#### Callback-ek

```python
def callback_target(self, msg: PoseStamped):
    self.target_pose = msg

def callback_odom(self, msg: Odometry):
    self.odom = msg
```

**Egyszerű tárolás**: az aktuális cél és odometria frissítése.

---

#### Timer callback – szabályozási ciklus

```python
def timer_callback(self):
    if not self.target_pose or not self.odom:
        return
```

**Biztonsági ellenőrzés**: ha nincs cél vagy odometria, nem szabályozunk (elkerüljük a hibát).

---

#### Távolság és szögek számítása

```python
dx = self.target_pose.pose.position.x - self.odom.pose.pose.position.x
dy = self.target_pose.pose.position.y - self.odom.pose.pose.position.y
linear_error = math.sqrt(dx**2 + dy**2)
```

**Linear error (távolság)**: Euclidean távolság a céltól.

```python
target_yaw = math.atan2(dy, dx)
```

**Target yaw (célszög)**: melyik irányba kell néznie a robotnak, hogy a cél felé nézzen.
- `atan2(dy, dx)`: arctangens függvény, amely minden síknegyedben helyes eredményt ad.

---

#### Quaternion → Euler konverzió

```python
from tf_transformations import euler_from_quaternion

q = [self.odom.pose.pose.orientation.x,
     self.odom.pose.pose.orientation.y,
     self.odom.pose.pose.orientation.z,
     self.odom.pose.pose.orientation.w]
current_yaw = euler_from_quaternion(q)[2]  # [roll, pitch, yaw]
```

**Célból is:**
```python
g_goal = [self.target_pose.pose.orientation.x,
          self.target_pose.pose.orientation.y,
          self.target_pose.pose.orientation.z,
          self.target_pose.pose.orientation.w]
goal_yaw = euler_from_quaternion(g_goal)[2]
```

**Miért kell?**
- Az odometria és cél quaternion-ban van.
- Szöghiba számításához Euler-szögek kellenek.

---

#### Longitudinális szabályozó (előre/hátra)

```python
linear_output = 0.0
if linear_error > self.max_speed:
    linear_output = self.max_speed  # Saturáció
elif linear_error > 0.1:
    linear_output = linear_error  # Proporcionális
```

**Logika:**
- Ha távolság > max_speed → teljes sebességgel megyünk (saturáció).
- Ha távolság > 0.1 m → sebességgel arányos a távolság (P szabályozó).
- Ha távolság < 0.1 m → megállunk (cél elérve).

**Miért nincs itt PID?**
- Egyszerű P szabályozó: sebesség = távolság (ha kicsi).
- Stabilitáshoz elég, mert nincs túllendülés (robot nem tud "túllőni" a célon).

---

#### Laterális szabályozó (forgatás)

```python
if linear_output > 0.0:
    angular_error = target_yaw - current_yaw
else:
    angular_error = goal_yaw - current_yaw
```

**Két fázis:**
1. **Mozgás közben**: nézzen a cél felé (`target_yaw`).
2. **Megérkezéskor**: álljon be a cél orientációba (`goal_yaw`).

**Példa:**
- Cél: (5, 5), orientáció: 90° (észak).
- Robot: (0, 0), orientáció: 0° (kelet).
- **Mozgás közben**: forgás ~45°-ra (cél irányába).
- **Megérkezve**: forgás 90°-ra (cél orientációba).

---

#### Szöghiba normalizálása

```python
corrected_angle = math.atan2(math.sin(angular_error), math.cos(angular_error))
```

**Miért kell?**
- Szögek ciklikusak: 180° = -180°.
- Például: 350° - 10° = 340° (helyesen: -20°).
- `atan2(sin(θ), cos(θ))` → [-π, π] tartományra normalizálja.

---

#### PID számítás

```python
self.integral += corrected_angle * self.dt
derivative = (corrected_angle - self.previous_error) / self.dt

angular_output = (self.Kp * corrected_angle + 
                  self.Ki * self.integral + 
                  self.Kd * derivative)
```

**P tag**: `Kp * hiba` – azonnali reakció.  
**I tag**: `Ki * ∫hiba·dt` – felhalmozott hiba korrekciója.  
**D tag**: `Kd * Δhiba/Δt` – hiba változás mértéke (csillapítás).

**Saturáció (korlátozás):**
```python
angular_output = max(min(angular_output, self.max_angular_speed), 
                     -self.max_angular_speed)
```

Biztosítjuk, hogy a szögsebesség a megengedett tartományban legyen.

---

#### Sebesség publikálása

```python
cmd = Twist()
cmd.linear.x = linear_output
cmd.angular.z = angular_output
self.pub_cmd.publish(cmd)

self.previous_error = corrected_angle
```

**Twist üzenet:**
- `linear.x`: előre/hátra sebesség (m/s).
- `angular.z`: szögsebesség (rad/s).

---

## 🚀 Launch fájl

[launch/gyak7.launch.xml](launch/gyak7.launch.xml)

```xml
<launch>
    <!-- Robot szimulátor -->
    <node pkg="gyak7" exec="diff_robot" name="diff_robot" output="screen"/>

    <!-- PID szabályozó -->
    <node pkg="gyak7" exec="control" name="control" output="screen">
        <param name="Kp" value="10.0"/>
        <param name="Ki" value="0.0"/>
        <param name="Kd" value="0.0"/>
        <param name="max_speed" value="0.2"/>
        <param name="max_angular_speed" value="0.5"/>
    </node>

    <!-- RViz -->
    <node pkg="rviz2" exec="rviz2" name="rviz2" args="-d $(find-pkg-share gyak7)/rviz/gyak7.rviz"/>
</launch>
```

**Mit indít?**
1. `diff_robot`: robot szimulátor (kinematika).
2. `control`: PID szabályozó (póz követés).
3. `rviz2`: vizualizáció.

**Paraméter hangolás:**
- `Kp = 10.0`: agresszív forgatás (gyors reakció).
- `max_speed = 0.2`: lassú mozgás (stabil).
- `max_angular_speed = 0.5`: közepes forgatási sebesség.

---

## 🖼️ RViz használat

[rviz/gyak7.rviz](rviz/gyak7.rviz)

- Fix Frame: `odom`
- Megjelenítők:
  - `Grid` – referencia rács
  - `Odometry` – `/odom` (robot pozíció, piros nyíl)
  - `TF` – `odom → base_link` transzformáció
  - **`Pose`** – `/goal_pose` (célpozíció, piros nyíl)

**Interaktív célmegadás:**
1. Felül az eszköztáron: **"2D Goal Pose"**.
2. Kattints és húzd RViz-ben → célpozíció és orientáció beállítása.
3. A robot automatikusan elindul a cél felé!

---

## 📦 Build és futtatás

```bash
# Build
colcon build --packages-select gyak7

# Forrásold a környezetet
source install/setup.bash

# Indítás
ros2 launch gyak7 gyak7.launch.xml
```

**Mit látsz?**
- RViz megnyílik.
- Piros nyíl az origóban (robot).
- Használd a "2D Goal Pose" eszközt → kattints egy célpontra.
- A robot elfordul és elmegy a célhoz!

**Ellenőrzés:**
```bash
ros2 topic list
ros2 topic echo /odom
ros2 topic echo /cmd_vel
ros2 topic echo /goal_pose

# Manuális cél küldése
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped "{pose: {position: {x: 2.0, y: 2.0}}}"
```

---

## 🎓 Mit tanulsz ebből?

### 1. Differenciális robot kinematika
- Egyszerű 2D mozgás modellezése.
- `x, y, yaw` állapotváltozók frissítése.
- Sebesség → pozíció integráció.

### 2. PID szabályozás
- P, I, D tagok működése.
- Feedback loop: hiba → beavatkozás → korrekció.
- Paraméter hangolás (Kp, Ki, Kd).

### 3. Koordináta-rendszerek és TF
- `odom` (globális) vs. `base_link` (robot helyi).
- TF transzformációk publikálása.
- Quaternion ↔ Euler konverzió.

### 4. Póz követés
- Távolság és szöghiba számítása.
- Longitudinális és laterális szabályozás szétválasztása.
- Kétfázisú cél: pozíció + orientáció.

### 5. Timer-alapú szabályozás
- Fix frekvenciájú eseményvezérlés (10 Hz, 20 Hz).
- Discrete-time szabályozás (`dt` használata).

---

## 🔍 Gyakori hibák

| Probléma | Ok | Megoldás |
|---|---|---|
| Robot nem mozog | Nincs cél vagy nincs odometria | Adj meg célt RViz-ben vagy topic pub-bal |
| Oszcilláció (rezgés) | `Kp` túl nagy | Csökkentsd `Kp`-t (pl. 5.0 vagy 3.0) |
| Lassú reakció | `Kp` túl kicsi | Növeld `Kp`-t (pl. 15.0) |
| Túllendülés | Nincs D tag | Adj hozzá `Kd`-t (pl. 0.5) |
| Maradó hiba | Nincs I tag | Adj hozzá `Ki`-t (pl. 0.1) |
| TF hiba RViz-ben | `diff_robot` nem fut | Ellenőrizd, hogy a node fut-e |

---

## 💡 Gyakorlási ötletek

1. **Hangold a PID-et:**
   - Próbáld `Kp = 5.0`, `Kp = 15.0`, `Kp = 20.0`.
   - Adj hozzá `Kd = 1.0` → csökkenti az oszcillációt?
   - Adj hozzá `Ki = 0.1` → kiküszöböli a maradó hibát?

2. **Változtasd a sebességkorlátokat:**
   - `max_speed = 0.5` → gyorsabb mozgás.
   - `max_angular_speed = 1.0` → gyorsabb forgatás.

3. **Vizualizáld a hibát:**
   - Publikálj egy `Float64` üzenetet az `angular_error`-ral.
   - Plotold `rqt_plot`-tal.

4. **Path követés:**
   - Módosítsd a szabályozót, hogy több célpontot kövessen egymás után.

5. **Ütközéselkerülés:**
   - Integráld a `gyak6` LaserScan feldolgozót → állj meg akadálynál.

---

## 📚 Hasznos ROS2 parancsok

```bash
# Node-ok és topic-ok
ros2 node list
ros2 topic list
ros2 topic hz /odom
ros2 topic hz /cmd_vel

# TF fa megtekintése
ros2 run tf2_tools view_frames
# (generál egy frames.pdf fájlt)

# TF echo (odom → base_link)
ros2 run tf2_ros tf2_echo odom base_link

# Paraméterek
ros2 param list /control_node
ros2 param get /control_node Kp
ros2 param set /control_node Kp 15.0
```

---

## 🎯 Vizsgára felkészülés

### Amit tudnod kell:
- ✅ Mi az a differenciális robot kinematika?
- ✅ Kinematikai egyenletek: `x, y, yaw` frissítése.
- ✅ Mi az a PID szabályozó és hogyan működik?
- ✅ P, I, D tagok szerepe és hatása.
- ✅ Quaternion ↔ Euler konverzió.
- ✅ TF transzformációk: `odom → base_link`.
- ✅ Szöghiba normalizálása ([-π, π]).
- ✅ Longitudinális vs. laterális szabályozás.

### Vizsgán gyakori feladatok:
- PID szabályozó implementálása.
- Robot kinematika frissítése.
- TF publikálás.
- Szöghiba számítás és normalizálás.
- Paraméter hangolás (Kp, Ki, Kd hatásának magyarázata).

---

## 🏁 Összefoglalás

A `gyak7` csomag egy komplett robot vezérlési rendszert demonstrál: egy differenciális robot szimulátort és egy PID szabályozót. A kinematikai modell megmutatja, hogyan frissül a robot pozíciója sebesség parancsok alapján, míg a PID szabályozó példázza a feedback vezérlés alapjait. Az interaktív RViz célmegadással könnyen kísérletezhetsz különböző paraméterekkel és megértheted a szabályozás viselkedését.

**Sok sikert a tanuláshoz és a vizsgához!** 🚀
