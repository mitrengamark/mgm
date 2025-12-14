# gyak4 – TurtleBot3 szimuláció + Odom → Path vizualizáció

A gyak4 csomag egy komplett, „egy gombos” indítócsomag: elindít egy TurtleBot3 Gazebo világot, futtatja a `gyak3` csomag PathCreator node-ját (odometriából `nav_msgs/Path`-ot épít), és megnyit egy előre beállított RViz nézetet, ahol látszik az odometria, a lidar és a felépített path.

---

## Cél és működés
- Gazebo-ban elindul egy TurtleBot3 (alap világban), opcionális kezdőpozíció paraméterrel.
- A `gyak3/path.py` node feliratkozik az `/odom` topikra és folyamatosan építi a `/path` topikot (`nav_msgs/Path`).
- Az RViz konfigurációban (Fix frame: `odom`) megjelenik:
  - `Odometry` (topic: `/odom`),
  - `LaserScan` (topic: `/scan`),
  - `RobotModel` (`/robot_description`),
  - `Path` (topic: `/path`).
- Paraméterezés YAML-ból: path topik neve, maximális hossz, szimulált idő használata.

---

## Fájlstruktúra

```
mgm_gyak/gyak4/
├── CMakeLists.txt                      # ament_cmake alapú csomag (megosztott erőforrások telepítése)
├── package.xml                         # metaadatok + futásidejű függőségek (rclpy)
├── config/
│   └── path_param.yaml                 # PathCreator paraméterek (topic nevek, max méret, sim time)
├── launch/
│   └── gyak4.launch.xml                # Gazebo + Path node + RViz indítás
└── rviz/
    └── gyak4.rviz                      # RViz beállítások (odom, scan, path, robotmodel)
```

Hasznos hivatkozások a csomagban:
- [launch/gyak4.launch.xml](launch/gyak4.launch.xml)
- [config/path_param.yaml](config/path_param.yaml)
- [rviz/gyak4.rviz](rviz/gyak4.rviz)
- [CMakeLists.txt](CMakeLists.txt) · [package.xml](package.xml)

Megjegyzés: a PathCreator implementációja a [gyak3/path.py](../gyak3/gyak3/path.py) fájlban található.

---

## Launch: mit indít el?
Részlet: [launch/gyak4.launch.xml](launch/gyak4.launch.xml)

- `turtlebot3_gazebo` világ indítása (szimulált idő bekapcsolva):
  ```xml
  <include file="$(find-pkg-share turtlebot3_gazebo)/launch/turtlebot3_world.launch.py">
    <arg name="use_sim_time" value="true"/>
    <arg name="x_pose" value="$(var x_pose)"/>
    <arg name="y_pose" value="$(var y_pose)"/>
  </include>
  ```
  Indításkori argumentumok: `x_pose`, `y_pose` (alap: 1.0, 2.0) – a robot kezdeti pozíciója.

- PathCreator node indítása a `gyak3` csomagból, YAML paraméterezéssel:
  ```xml
  <node pkg="gyak3" exec="path.py" name="path_node" output="screen">
    <param from="$(find-pkg-share gyak4)/config/path_param.yaml"/>
  </node>
  ```

- RViz megnyitása az előre beállított nézettel:
  ```xml
  <node pkg="rviz2" exec="rviz2" name="rviz2" args="-d $(find-pkg-share gyak4)/rviz/gyak4.rviz"/>
  ```

---

## Paraméterezés (YAML)
Lásd: [config/path_param.yaml](config/path_param.yaml)

```yaml
/**:
  ros__parameters:
    path_topic_name: /path
    max_size: 500
    use_sim_time: true
```

- `path_topic_name`: a `nav_msgs/Path` publikálási topicja.
- `max_size`: a path maximális elemszáma; ha eléri, a legrégebbi ~20% törlésre kerül (trimmelés).
- `use_sim_time`: kapcsolja a szimulált időt (rosbag/Gazebo mellett szükséges, hogy az RViz és a node-ok szinkronban legyenek a `/clock`-kal).

---

## RViz beállítások
Részlet: [rviz/gyak4.rviz](rviz/gyak4.rviz)

- Fix Frame: `odom`.
- Megjelenítők:
  - `Grid` – referencia rács,
  - `Odometry` – `/odom` nyíllal és kovarianciával,
  - `LaserScan` – `/scan` (Best Effort),
  - `RobotModel` – `/robot_description` alapján,
  - `TF` – keretek ellenőrzésére,
  - `Path` – `/path` zöld vonal (Line Style: Lines).

Ha a Path nem látható, ellenőrizd, hogy valóban jön-e adat a `/path`-on és a fix frame helyes-e (`odom`).

---

## Előfeltételek
- A `gyak3` csomag legyen a workspace-ben buildelve (ebből jön a `path.py` node).
- Telepített TurtleBot3 csomagok (Gazebo-val): `turtlebot3_gazebo`, `turtlebot3_description`.
- Állítsd a TB3 modellt, például:
  ```bash
  export TURTLEBOT3_MODEL=burger
  ```

---

## Build és futtatás
A workspace gyökeréből:

```bash
# Buildeld a csomagokat (gyak3 + gyak4 is kell)
colcon build --packages-select gyak3 gyak4

# Forrásold a környezetet
source install/setup.bash

# Indítás alapértelmezett kezdőpozícióval
ros2 launch gyak4 gyak4.launch.xml

# Opcionális: indulási pozíció megadása
ros2 launch gyak4 gyak4.launch.xml x_pose:=0.0 y_pose:=0.0
```

Hasznos ellenőrzések:
```bash
ros2 topic list
ros2 topic echo /odom
ros2 topic echo /path

ros2 param list /path_node
ros2 param get /path_node use_sim_time
```

---

## Hibakeresés
| Jelenség | Ok | Megoldás |
|---|---|---|
| RViz nem mozog az idővel | Nincs szimulált idő | Ellenőrizd: `use_sim_time=true`, `/clock` jön-e Gazebo-ból |
| Nincs `/path` adat | `gyak3` nincs buildelve vagy a node nem fut | Buildeld a `gyak3`-at; nézd a `path_node` logját |
| TB3 modell nem jelenik meg | Hiányzó TB3 csomagok vagy `TURTLEBOT3_MODEL` | Telepítsd a csomagokat; exportáld a modellt |
| LaserScan üres | A világban nincs adat/érvényes sensor | Változtasd a világot / ellenőrizd a szenzortémákat |
| Fix frame hibás | RViz `Fixed Frame` nem `odom` | Állítsd `odom`-ra (összhangban az odometriával) |

---

## Mit tanulsz ebből?
- Launch fájlokban más csomag indítása (`<include>`) és argumentumok kezelése.
- Node paraméterezése YAML-ból (`<param from=...>` és `ros__parameters`).
- Szimulált idő helyes használata (Gazebo/rosbag → `/clock` → `use_sim_time`).
- RViz nézetek felépítése és Path megjelenítése.
- Csomagolás `ament_cmake`-kel, megosztott erőforrások telepítése (`config/`, `launch/`, `rviz/`).

---

## Rövid összefoglaló
A `gyak4` egy praktikus indítócsomag, amellyel azonnal kipróbálhatod egy mobil robot odometriájából épülő útvonal vizualizációját szimulációban. A `gyak3` PathCreator node-ját újrahasznosítja, YAML-ból paraméterezve, és mindezt egy kényelmes RViz nézettel mutatja meg.
