# Feladat2 - proba2 Package

## Cél
`/agent1/odom/ground_truth` Odometry feldolgozása:
- Pozíció x és y felcserélése vizuális megjelenítéshez.
- 10x10x20 cm-es téglatest (Marker) kirajzolása a jármű orientációjával.
- 0.2 Hz-től (5 s) publikálni az utolsó Odometry időbélyegét (`std_msgs/Header`).

## Fő node: `proba2_node`
Funkciók:
1. Feliratkozás: `/agent1/odom/ground_truth` (nav_msgs/Odometry)
2. Marker publikálás: `/swapped_box` (visualization_msgs/Marker)
   - Pozíció: `x' = y`, `y' = x`, `z' = z`
   - Méret (m): `scale.x = 0.20`, `scale.y = 0.10`, `scale.z = 0.10`
   - Orientáció: Odometry-ből átvéve (hosszú oldal előre)
3. Időbélyeg publikálás: `/last_odom_header` (std_msgs/Header) – timer 5.0 s.

## Build (Ubuntu 22.04, ROS 2 Humble)
```bash
source /opt/ros/humble/setup.bash
# Workspace src könyvtárba másold a csomagot, majd:
colcon build --packages-select proba2 --symlink-install
source install/setup.bash
```

## Futtatás
```bash
ros2 launch proba2 proba2.launch.xml
```

Bag lejátszás (külön terminál):
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 bag play /path/to/bag --clock
```

## Ellenőrzés
```bash
ros2 topic list
ros2 topic echo /swapped_box
ros2 topic echo /last_odom_header
ros2 node info /proba2_node
```

## Kódrészlet – Marker generálás
```python
ox = msg.pose.pose.position.x
oy = msg.pose.pose.position.y
swapped_x = oy
swapped_y = ox
marker.scale.x = 0.20  # hosszú oldal előre (x)
marker.scale.y = 0.10
marker.scale.z = 0.10
marker.pose.orientation = msg.pose.pose.orientation
```

## Miért x ↔ y csere?
A feladat megnevezett átalakítási lépést kér – ilyen transzformációkkal gyakran koordinátarendszer-eltéréseket szimulálunk vagy ellenőrző logikát implementálunk.

## Hibák / Diagnosztika
- Ha nincs Odometry: a timer csak figyelmeztetést logol.
- Import hibák Mac-en: ROS 2 hiánya miatt normális.
- Ha RViz-ben nem látszik a Marker: ellenőrizd a topicot (`/swapped_box`) és a Fixed Frame-et (`map` vagy `odom`).

## Függőségek (package.xml)
```
rclpy, nav_msgs, geometry_msgs, std_msgs, visualization_msgs, tf_transformations
```

## További ötletek
- Szín parametrizálása launch paramként.
- Marker lifetime beállítása animációhoz.
- Header frame_id módosítása (pl. "swapped_frame").

---
Sok sikert a ZH-hoz! 🚀
