# gyak3 – Odom → Path vizualizáció rosbaggel és RViz-zel

Ez a csomag egy egyszerű, de vizsgán is hasznos ROS 2 példa: az odometriából (Odometry) folyamatosan épít egy útvonalat (Path), majd azt publikálja és RViz-ben megjeleníti. A csomag mellé egy rosbag is tartozik, így szenzor/robot nélkül is kipróbálható.

---

## Mi a cél?
- Odometriából (`/odom`) menet közben egy `nav_msgs/Path` építése és publikálása (`/path`).
- A path méretének karbantartása (régi pontok trimmelése).
- Egyszerű futtatás rosbag visszajátszással és előre beállított RViz nézettel.

---

## Fájlstruktúra

```
mgm_gyak/gyak3/
├── CMakeLists.txt                  # CMake-alapú Python csomag telepítés
├── package.xml                     # Csomag metaadatok (ament_cmake)
├── gyak3/
│   ├── __init__.py
│   └── path.py                    # PathCreator node (Odom → Path)
├── launch/
│   └── gyak3.launch.xml           # Rosbag lejátszás + node + RViz indítás
├── rviz/
│   └── gyak3.rviz                 # RViz konfiguráció (/odom, /scan, Fixed: odom)
└── bag/                           # Minta rosbag (/odom, /scan, stb.)
    ├── metadata.yaml
    └── rosbag2_*.db3
```

Hasznos hivatkozások a repo-ban:
- [gyak3/path.py](gyak3/path.py)
- [launch/gyak3.launch.xml](launch/gyak3.launch.xml)
- [rviz/gyak3.rviz](rviz/gyak3.rviz)
- [CMakeLists.txt](CMakeLists.txt)
- [package.xml](package.xml)

---

## A node működése (PathCreator)
A node neve: `odom_path` (a kódban a `PathCreator` osztály). Fő feladata, hogy minden beérkező odometriát `PoseStamped`-dé alakítson és felfűzzön egy `Path` üzenetbe, majd ezt folyamatosan publikálja.

Főbb lépések (röviden):
1. Feliratkozás az odometriára (`/odom`, `nav_msgs/Odometry`).
2. Új üzenetnél: a header és a pozíció/orientáció átmásolása egy `PoseStamped`-be.
3. Hozzáfűzés egy `nav_msgs/Path` objektum `poses` listájához.
4. Ha elérte a maximális elemszámot, a legrégebbi ~20% törlése (trimmelés).
5. A frissített `Path` publikálása a beállított topicon (alapból `/path`).

Kódrészlet és magyarázat: [gyak3/path.py](gyak3/path.py)

```py
class PathCreator(Node):
    def __init__(self):
        super().__init__('odom_path')
        self.declare_parameter('path_topic_name', '/path')
        self.declare_parameter('odom_topic_name', '/odom')
        self.declare_parameter('max_size', 1500)
        # ...
        self.path_pub = self.create_publisher(Path, path_topic_name, 1)
        self.odom_sub = self.create_subscription(Odometry, odom_topic_name, self.odom_cb, 1)

    def odom_cb(self, msg: Odometry):
        if (len(self.path.poses) >= self.max_size) and (self.max_size > 0):
            del self.path.poses[0:int(self.max_size * 0.2)]  # régi pontok trimmelése
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.path.header = msg.header
        self.path.poses.append(pose)
        self.path_pub.publish(self.path)
```

### Paraméterek
- `path_topic_name` (alapértelmezett: `/path`): ide publikálja a `nav_msgs/Path`-ot.
- `odom_topic_name` (alapértelmezett: `/odom`): innen érkezik a `nav_msgs/Odometry`.
- `max_size` (alapértelmezett: `1500`): ennyi `PoseStamped` lehet maximum a path-ban. `0` esetén nincs limit.

### Használt üzenetek
- Bemenet: `nav_msgs/Odometry`
- Kimenet: `nav_msgs/Path` (benne: `geometry_msgs/PoseStamped` elemek)

### Miért fontos a header?
A `header` átvétele biztosítja, hogy a `Path` és az `Odometry` ugyanarra a koordinátarendszerre és időbélyegre vonatkozzon, így a megjelenítés és a szinkronizáció helyes lesz (különösen szimulált időnél).

---

## Launch fájl – rosbag + node + RViz
Lásd: [launch/gyak3.launch.xml](launch/gyak3.launch.xml)

Mit indít el?
- Rosbag lejátszást a csomag `bag/` mappájából (publikál `/clock`-ot és adatokat, pl. `/odom`, `/scan`).
- A `PathCreator` node-ot a fenti paraméterekkel, és `use_sim_time=true` beállítással (szimulált idő használata a bag-hez).
- RViz2-t az előre beállított nézettel ([rviz/gyak3.rviz](rviz/gyak3.rviz)).

```xml
<node pkg="gyak3" exec="path.py" name="path_node" output="screen">
  <param name="use_sim_time" value="true"/>
  <param name="path_topic_name" value="/path"/>
  <param name="odom_topic_name" value="/odom"/>
  <param name="max_size" value="500"/>
</node>
```

Megjegyzés: az RViz file alapból `Odometry` és `LaserScan` megjelenítőt tartalmaz. A `Path` megjelenítéséhez érdemes egy `Path` displayt hozzáadni és a topicot `/path`-ra állítani, ha az még nincs konfigurálva.

---

## Build és futtatás
A csomag CMake-alapú, Python node-dal. A `path.py` telepített futtathatóként érhető el.

```bash
# 1) Build (a workspace gyökeréből)
colcon build --packages-select gyak3

# 2) Környezet betöltése
source install/setup.bash

# 3/a) Launch: rosbag + node + RViz együtt
ros2 launch gyak3 gyak3.launch.xml

# 3/b) Csak a node indítása (külön rosbag play-jel)
ros2 run gyak3 path.py
# (külön terminálban):
ros2 bag play <útvonal_a_baghez>  # ha nem a csomaghoz tartozó baget használod
```

Hasznos ellenőrzések futás közben:
```bash
ros2 topic list
ros2 topic echo /odom   # bemenet
ros2 topic echo /path   # kimenet

ros2 param list /path_node
ros2 param get /path_node use_sim_time
```

---

## RViz használat
- Fixed Frame: `odom` (a konfigurációban így van beállítva).
- Megjelenítők a configban: `Grid`, `Odometry` (`/odom`), `LaserScan` (`/scan`).
- A `Path` megjelenítéséhez adj hozzá egy `Path` display-t és állítsd a `Topic`-ot `/path`-ra (szín/ vastagság tetszőlegesen módosítható).

Parancs RViz külön indításához (ha kell):
```bash
rviz2 -d $(ros2 pkg prefix gyak3)/share/gyak3/rviz/gyak3.rviz
```

---

## Tuning és tippek
- `max_size`: növeld/csökkentsd a path részletességét és memóriahasználatát. `0` = nincs limit, de nőhet a memóriaigény.
- Saját adatokra: állítsd át az `odom_topic_name`-et a rendszeredhez illőre (pl. `/robot1/odom`).
- Path ritkítás: ha zajos a pozíció, előfeldolgozásként (node-on belül) bevezethetsz idő- vagy távolság-alapú mintavételezést.
- Rosbag: ha túl gyors/ lassú a lejátszás, indítsd külön a play-t saját `--rate` beállítással.

---

## Gyakori hibák
| Jelenség | Ok | Megoldás |
|---|---|---|
| RViz-ben nem látszik a Path | Nincs Path display hozzáadva, vagy rossz topic | Adj hozzá `Path` display-t `/path` topic-kal |
| Nincs adat a `/odom`-on | Rosbag nem fut / nincs robot | Indíts rosbag play-t vagy a szenzorokat |
| Idő szinkron gondok | Nincs `use_sim_time=true` bag visszajátszásnál | Kapcsold be a paramétert (launch megteszi) |
| Túl nagy memóriahasználat | `max_size` túl nagy / 0 | Csökkentsd a `max_size`-t |

---

## Tanulási ellenőrzőlista
- Értem a `Publisher`/`Subscriber` mintát és a callback működését.
- Tudom, mi a `nav_msgs/Odometry` és a `nav_msgs/Path` szerkezete.
- Tudom, miért fontos a `header` átvétele (idő/keret szinkron).
- Tudom, hogyan kell `use_sim_time`-ot használni rosbag-gel.
- Tudok RViz-ben `Path`-ot megjeleníteni és testre szabni.

---

## Rövid összefoglaló
A `gyak3` csomag az odometria-alapú útvonalépítés és vizualizáció gyakorlására fókuszál. A `PathCreator` node felépíti a bejárt útvonalat, a launch fájl rosbaggel és RViz-zel együtt pedig gyors kipróbálást tesz lehetővé valódi robot nélkül is.
