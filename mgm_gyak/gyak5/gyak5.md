# gyak5 – RViz Vizualizáció Markerekkel

A `gyak5` csomag az ROS 2 **Marker** és **MarkerArray** üzenetek használatát mutatja be. Odometriából valós időben 3D vizuális objektumokat (marker) hoz létre RViz megjelenítéshez: kockát az aktuális pozícióban, gömbökből álló útvonalat, és egy bounding box-ot, amely a bejárt terület határait jelöli.

---

## 🎯 Mi a cél?

- Megtanulni, hogyan használjunk **`visualization_msgs/Marker`** és **`MarkerArray`** üzeneteket.
- Különböző marker típusok: `CUBE`, `SPHERE`, különböző színekkel és méretekkel.
- Odometriából automatikusan vizuális objektumok generálása (pozíció, path, bounding box).
- RViz-ben valós időben látni az egyedi vizualizációkat.

---

## 📁 Fájlstruktúra

```
mgm_gyak/gyak5/
├── package.xml                   # Csomag metaadatok (ament_python)
├── setup.py                      # Python entry points (test_viz futtatható)
├── setup.cfg                     # Python setuptools konfig
├── gyak5/
│   ├── __init__.py
│   └── test_viz.py              # VizPub node: Marker publikálás
├── launch/
│   └── gyak5.launch.xml         # Rosbag play + node + RViz indítás
├── rviz/
│   └── gyak5.rviz               # RViz konfiguráció (MarkerArray display)
├── resource/
│   └── gyak5
└── test/                        # Teszt fájlok (copyright, flake8, pep257)
```

Főbb fájlok:
- [gyak5/test_viz.py](gyak5/test_viz.py) – a vizualizációs node
- [launch/gyak5.launch.xml](launch/gyak5.launch.xml) – indító fájl
- [rviz/gyak5.rviz](rviz/gyak5.rviz) – RViz beállítások

---

## 🔧 A VizPub node működése

### Áttekintés

[gyak5/test_viz.py](gyak5/test_viz.py)

A `VizPub` osztály feliratkozik az odometria topicra (`/odom`) és minden beérkező üzenetből több vizuális markert készít:

1. **CUBE marker** (kék kocka) – az aktuális robot pozíció.
2. **SPHERE markerek** (színes gömbök) – korábbi pózok path-ként, ritkítva.
3. **CUBE marker** (piros bounding box) – a bejárt terület határai.

### Részletes kód magyarázat

#### Inicializálás

```python
class VizPub(Node):
    def __init__(self):
        super().__init__('test_viz')
        
        odom_topic_name = self.declare_parameter('odom_topic_name', '/odom').value
        
        self.sub = self.create_subscription(Odometry, odom_topic_name, self.callback_odom, 1)
        self.pub = self.create_publisher(MarkerArray, "/viz", 1)
        
        self.pose_list = []
```

**Mit csinál?**
- Node neve: `test_viz`
- Paraméter: `odom_topic_name` (alapértelmezett: `/odom`)
- Subscriber: feliratkozik az odometriára
- Publisher: `MarkerArray` üzeneteket publikál a `/viz` topicra
- `pose_list`: itt tároljuk a korábbi pózokat (path építéséhez)

---

#### Odometria callback – marker generálás

```python
def callback_odom(self, msg: Odometry):
    marker_array = MarkerArray()
    
    # 1. Aktuális pozíció marker (CUBE)
    marker = Marker()
    marker.header = msg.header
    marker.ns = "cube"
    marker.id = 0
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
```

**`MarkerArray`**: egy lista, amely több `Marker`-t tud tárolni. Egyetlen üzenetben több vizuális objektumot publikálhatunk.

**`Marker` mezők:**
- `header`: időbélyeg és koordinátarendszer (pl. `odom`)
- `ns` (namespace): markerek csoportosítása (azonos ID-k különböző namespace-ekben nem ütköznek)
- `id`: egyedi azonosító a namespace-en belül
- `type`: marker típusa (`CUBE`, `SPHERE`, `ARROW`, `LINE_STRIP`, stb.)
- `action`: `ADD` = hozzáadás/frissítés, `DELETE` = törlés

---

#### Pozíció, méret, szín

```python
marker.pose = msg.pose.pose

marker.scale.x = 0.2
marker.scale.y = 0.2
marker.scale.z = 0.4

marker.color.a = 1.0  # Alpha (átlátszóság): 1.0 = teljesen átlátszatlan
marker.color.r = 0.0
marker.color.g = 0.0
marker.color.b = 1.0  # Kék szín

marker.lifetime = rclpy.duration.Duration(seconds=0.1).to_msg()
```

**Pozíció és orientáció:**
- `marker.pose`: a robot aktuális pózának átvétele az odometriából

**Méret:**
- `scale.x/y/z`: a marker mérete méterben (CUBE esetén szélesség/mélység/magasság)

**Szín (RGBA):**
- `r/g/b`: vörös/zöld/kék komponensek (0.0–1.0)
- `a`: alpha/átlátszatlanság (0.0 = teljesen átlátszó, 1.0 = teljesen átlátszatlan)

**Lifetime (élettartam):**
- `0.1` sec: ha ennyi idő alatt nem frissül a marker, RViz automatikusan eltávolítja
- Ha `lifetime = 0`, a marker örökké megmarad (amíg nem törlöd)

---

#### Path markerek – gömbökből álló útvonal

```python
def path_publisher(self, msg: Odometry, marker_array_: MarkerArray):
    self.pose_list.append(msg.pose.pose)
    
    marker_path = Marker()
    marker_path.header = msg.header
    marker_path.ns = "path"
    marker_path.type = Marker.SPHERE
    marker_path.action = Marker.ADD
    
    marker_path.scale.x = 0.1
    marker_path.scale.y = 0.1
    marker_path.scale.z = 0.1
    
    marker_path.lifetime = rclpy.duration.Duration(seconds=0.1).to_msg()
```

**Működés:**
1. Minden új póz hozzáadódik a `pose_list`-hez.
2. Végigiterálunk a listán **ritkítva** (csak minden 30. elem).
3. Minden kiválasztott pózhoz egy `SPHERE` marker készül.

**Ritkítás (teljesítmény optimalizálás):**
```python
for i in range(0, len(self.pose_list), 30):
    marker_path.id = i
    marker_path.pose = self.pose_list[i]
    
    marker_path.color.a = random.uniform(0.2, 1.0)
    marker_path.color.r = random.uniform(0.0, 1.0)
    marker_path.color.g = random.uniform(0.0, 1.0)
    marker_path.color.b = random.uniform(0.0, 1.0)
    
    marker_array_.markers.append(copy.deepcopy(marker_path))
```

**Miért kell a `copy.deepcopy()`?**
- Python-ban a referenciák megosztottak – ha nem másolunk, minden marker ugyanaz az objektum lesz.
- `deepcopy` új objektumot hoz létre, így minden marker saját színnel/pozícióval rendelkezik.

**Véletlen színek:**
- `random.uniform(0.0, 1.0)`: véletlen érték a megadott tartományban
- Így minden gömb más színű lesz → könnyen követhető a path

---

#### Bounding box – bejárt terület határa

```python
def rectangle_publisher(self, marker_array_: MarkerArray):
    x_min = 999.0
    x_max = -999.0
    y_min = 999.0
    y_max = -999.0
    
    for i in range(0, len(self.pose_list) - 1):
        i_pose = self.pose_list[i]
        
        if (x_min > i_pose.position.x):
            x_min = i_pose.position.x
        if (x_max < i_pose.position.x):
            x_max = i_pose.position.x
        if (y_min > i_pose.position.y):
            y_min = i_pose.position.y
        if (y_max < i_pose.position.y):
            y_max = i_pose.position.y
```

**Működés:**
1. Végigmegyünk az összes pózón.
2. Megkeressük a minimális és maximális X és Y koordinátákat.
3. Ezekből kiszámítjuk a bounding box közepét és méretét.

```python
marker_rectangle = Marker()
marker_rectangle.header.frame_id = "odom"
marker_rectangle.header.stamp = self.get_clock().now().to_msg()

marker_rectangle.ns = "rect"
marker_rectangle.id = 0
marker_rectangle.type = Marker.CUBE
marker_rectangle.action = Marker.ADD

marker_rectangle.color.a = 0.2  # Átlátszó
marker_rectangle.color.r = 1.0  # Piros

marker_rectangle.pose.position.x = (x_min + x_max) / 2.0
marker_rectangle.pose.position.y = (y_min + y_max) / 2.0
marker_rectangle.pose.orientation.w = 1.0

marker_rectangle.scale.x = abs(x_max - x_min)
marker_rectangle.scale.y = abs(y_max - y_min)
marker_rectangle.scale.z = 0.05  # Vékony lap
```

**Pozíció:** a bounding box középpontja (átlag).  
**Méret:** szélesség = `x_max - x_min`, magasság = `y_max - y_min`.  
**Alpha = 0.2:** átlátszó → látszik alatta a path/robot.

---

## 🚀 Launch fájl

[launch/gyak5.launch.xml](launch/gyak5.launch.xml)

```xml
<launch>
    <!-- Rosbag play (gyak3 bag-ből) -->
    <executable
        cmd="ros2 bag play --clock 1000 $(find-pkg-share gyak3)/bag"
        output="screen"
        name="rosbag_play"
        shell="true"
    />

    <!-- VizPub node -->
    <node pkg="gyak5" exec="test_viz" name="visualization" output="screen">
        <param name="odom_topic_name" value="/odom"/>
    </node>

    <!-- RViz -->
    <node pkg="rviz2" exec="rviz2" name="rviz2" args="-d $(find-pkg-share gyak5)/rviz/gyak5.rviz"/>
</launch>
```

**Mit indít?**
1. Rosbag lejátszás (ugyanaz, mint gyak3-nál – `/odom`, `/scan` adatok).
2. `test_viz` node (marker generálás).
3. RViz előre konfigurált nézettel.

---

## 🖼️ RViz beállítások

[rviz/gyak5.rviz](rviz/gyak5.rviz)

- Fix Frame: `odom`
- Megjelenítők:
  - `Grid` – referencia rács
  - `Odometry` – `/odom` (nyíl ikon)
  - `LaserScan` – `/scan` (lidar pontok)
  - **`MarkerArray`** – `/viz` topic (itt jelennek meg a markereink!)
  - `RobotModel` – robot vizualizáció (ha van `/robot_description`)

**Fontos:** a `MarkerArray` display topic-ja: `/viz` – ide publikál a node.

---

## 📦 Build és futtatás

```bash
# Build (workspace gyökeréből)
colcon build --packages-select gyak5

# Forrásold a környezetet
source install/setup.bash

# Indítás
ros2 launch gyak5 gyak5.launch.xml
```

**Mit látsz RViz-ben?**
- Kék kocka: aktuális robot pozíció (0.2×0.2×0.4 m)
- Színes gömbök: path (minden 30. póz)
- Piros átlátszó téglalap: bounding box

**Ellenőrzés:**
```bash
ros2 topic list
ros2 topic echo /viz

ros2 run rviz2 rviz2  # külön RViz indítás
```

---

## 🎓 Mit tanulsz ebből?

### 1. Marker típusok
- `CUBE`, `SPHERE`, `ARROW`, `CYLINDER`, `LINE_STRIP`, `POINTS`, stb.
- Minden típusnak más a `scale` jelentése.

### 2. Namespace (ns) és ID
- Namespace: markerek csoportosítása (pl. "cube", "path", "rect").
- ID: egyedi azonosító a namespace-en belül.
- Ugyanaz az ID+namespace kombináció **frissíti** a meglévő markert.

### 3. Lifetime
- Rövid lifetime (0.1 sec) → marker csak akkor látható, ha folyamatosan frissül.
- Hasznos mozgó/dinamikus objektumokhoz.

### 4. Szín és átlátszóság
- RGBA: vörös/zöld/kék/alpha.
- Alpha = 0.0: teljesen átlátszó → láthatatlan.
- Alpha = 1.0: teljesen átlátszatlan → tömör.

### 5. Deep copy
- Python referenciák megosztottak → `copy.deepcopy()` új objektumot hoz létre.
- Nélküle minden marker ugyanaz lenne.

### 6. Teljesítmény
- Sok marker (pl. 1000+) lassíthatja RViz-t.
- Ritkítás (pl. minden 30. elem) optimalizálja a megjelenítést.

---

## 🔍 Gyakori hibák

| Probléma | Ok | Megoldás |
|---|---|---|
| Marker nem látszik RViz-ben | Nincs `MarkerArray` display vagy rossz topic | Adj hozzá `MarkerArray` display-t `/viz` topickal |
| Marker túl kicsi/nagy | `scale` értéke nem megfelelő | Állítsd be a `scale.x/y/z` értékeket |
| Minden marker ugyanaz | Nincs `copy.deepcopy()` | Használj deep copy-t a hozzáadásnál |
| Marker azonnal eltűnik | Lifetime túl rövid | Növeld a `lifetime` értékét vagy állítsd 0-ra |
| Fix frame hiba | RViz fixed frame nem egyezik | Állítsd `odom`-ra (vagy a marker header frame-jével egyezően) |

---

## 💡 Gyakorlási ötletek

1. **Változtasd a marker típusát:** próbáld ki az `ARROW`, `CYLINDER`, `LINE_STRIP` típusokat.
2. **Állítsd át a színeket:** készíts monokróm path-t vagy rainbow effektet.
3. **Változtasd a ritkítást:** próbáld minden 10. vagy 50. elemmel.
4. **Adj hozzá szöveges markert:** `TEXT_VIEW_FACING` típussal írhatsz ki szöveget a térben.
5. **Animálj!** Változtasd a marker méretét/színét időben (pl. pulzálás).

---

## 📚 Hasznos linkek

- [visualization_msgs/Marker dokumentáció](http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/Marker.html)
- [RViz Marker típusok](http://wiki.ros.org/rviz/DisplayTypes/Marker)
- [MarkerArray példák](http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Basic%20Shapes)

---

## 🎯 Vizsgára felkészülés

### Amit tudnod kell:
- ✅ Mi az a `Marker` és `MarkerArray`?
- ✅ Hogyan állítsd be a pozíciót, méretet, színt?
- ✅ Mi a különbség a különböző marker típusok között?
- ✅ Mire jó a `lifetime`?
- ✅ Miért kell `copy.deepcopy()`?
- ✅ Hogyan csoportosítsd a markereket namespace-szel?

### Vizsgán gyakori:
- Készíts egyszerű marker publikálást.
- Változtasd a marker típusát/színét/méretét.
- Jelenítsd meg az robot környezetében lévő objektumokat (pl. akadályok).

---

## 🏁 Összefoglalás

A `gyak5` csomag a vizualizációs markerek használatát mutatja be. Az odometriából valós időben épülő vizuális objektumok (kocka, gömbök, bounding box) segítségével könnyen követhető a robot mozgása és a bejárt terület. Ez a tudás alapvető a ROS-alapú robot fejlesztésnél, ahol gyakran szükséges egyedi vizualizációk készítése (pl. tervezett útvonal, észlelt objektumok, célpontok).

**Boldog tanulást!** 🚀
