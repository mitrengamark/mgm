# Gyakorlatok Tartalma – Gyors Áttekintő

## 📦 **gyak2** – Alapvető Publisher és Subscriber
**Mit találsz benne:**
- Egyszerű String üzenet publikálása (publisher node)
- String üzenet fogadása (subscriber node – hiányos példa, javítandó)
- Timer használat (1 Hz-es periodikus publikálás)
- Logger szintek (info, warn, error)
- Alapvető Node struktúra

**Mire jó:**
- ROS2 alapok gyakorlása
- Publisher/Subscriber minta megértése
- QoS beállítások alapjai

---

## 🛤️ **gyak3** – Path generálás Odometriából
**Mit találsz benne:**
- Odometry → Path konverzió
- PoseStamped lista építése
- Path trimmelés (max_size paraméter alapján)
- Paraméter kezelés (declare_parameter, get_parameter)

**Mire jó:**
- Navigációs path építés robot mozgásából
- Path üzenet típus használata
- Paraméterek használata node-okban

---

## 🎨 **gyak5** – Vizualizációs Markerek (RViz)
**Mit találsz benne:**
- MarkerArray használat (több marker egyidejű publikálása)
- CUBE marker (aktuális pozíció)
- SPHERE marker lista (path pontok)
- Bounding box számítás és vizualizáció
- Random színek generálása
- Lifetime beállítás

**Mire jó:**
- RViz vizualizációk készítése
- Marker típusok megismerése
- Dinamikus objektumok megjelenítése

---

## 📡 **gyak6** – LaserScan Feldolgozás
**Mit találsz benne:**
- LaserScan → Descartes koordináták (polár → kartéziánus)
- PointCloud2 generálás
- SPHERE_LIST marker (összes scan pont)
- Legközelebbi pont detektálás
- Range szűrés (valid tartomány)

**Mire jó:**
- LiDAR/LaserScan adatok feldolgozása
- Point cloud kezelés
- Scan alapú objektum detektálás

---

## 🤖 **gyak7** – Differenciális Robot Szimulátor + PID Controller
**Mit találsz benne:**

### `diff_robot.py`:
- Differenciális kinematika (x, y, yaw)
- Twist (cmd_vel) parancsok fogadása
- Odometry publikálás
- TF broadcasting (odom → base_link)
- Quaternion konverzió

### `control.py`:
- PID szabályozó implementáció
- Goal pose követés
- Longitudinális és laterális vezérlés
- Kp, Ki, Kd paraméterek
- Sebességkorlátok

**Mire jó:**
- Robot szimuláció alapjai
- PID controller implementáció
- Pózkövetés algoritmusa

---

## 🔄 **gyak8** – TF Transformációk
**Mit találsz benne:**
- TF Buffer és Listener használat
- Static TF Broadcaster (base_link → safety_left)
- can_transform ellenőrzés
- lookup_transform lek

érdezés
- Odometry publikálás TF-ből

**Mire jó:**
- Koordináta-rendszer transzformációk
- TF fa használata
- Statikus és dinamikus TF-ek

---

## 🌐 **gyak9** – Multi-Robot LaserScan Aggregálás
**Mit találsz benne:**
- Több robot scan-jeinek összegyűjtése (namespace-ekkel)
- TF transzformáció (robot frame → map)
- do_transform_cloud (PointCloud2 transzformálás)
- PointCloud2 merge (több robot pontjainak egyesítése)
- Dictionary alapú ponttárolás (frame_id szerint)

**Mire jó:**
- Multi-robot rendszerek
- Namespace kezelés
- Globális térkép építés több robotból

---

## 🚗 **gyak10** – Ackermann Robot + Pure Pursuit
**Mit találsz benne:**

### `ack_robot.py`:
- Ackermann steering kinematika
- Wheelbase paraméter
- Statikus TF (base_link → front_wheel)
- Kormányzott robot szimuláció

### `pure_pursuit.py`:
- Pure Pursuit path követés algoritmus
- Legközelebbi pont keresés path-on
- Lookahead pont számítás
- Steering angle kalkuláció
- Marker vizualizáció (closest + lookahead)

**Mire jó:**
- Ackermann (autószerű) robot vezérlés
- Path tracking algoritmusok
- Lookahead alapú navigáció

---

## 💾 **gyak11** – Path Mentés és Betöltés Fájlból
**Mit találsz benne:**

### `save_path.py`:
- Odometry mentés text fájlba
- Távolság alapú ritkulás
- TF transzformáció (map frame-be)
- Quaternion → Yaw konverzió (fokokban)
- CSV formátum (x, y, z, yaw)

### `pub_path.py`:
- Path betöltés text fájlból
- PoseStamped lista generálás
- Yaw (fok) → Quaternion
- Bounding box marker (min/max pontok)
- Opcionális TF transzformáció

**Mire jó:**
- Path perzisztencia (mentés/betöltés)
- Offline path újrajátszás
- Path adatok exportálása/importálása

---

## 🎓 **gyak12** – ZH Feladat Példák
**Mit találsz benne:**

### `node1.py`:
- Odometry alapú távolság számolás
- Feltételes publikálás (egész rész változás)
- Float64 üzenet használat
- Akkumulált távolság követés

### `node2.py`:
- LaserScan → PointCloud2
- Súlyponti pont (centroid) számítás
- Marker vizualizáció (SPHERE)
- TF transzformáció (scan → map)
- PoseStamped publikálás glob

ális frame-ben

**Mire jó:**
- ZH feladatok gyakorlása
- Összetett feldolgozási pipeline
- Multi-topic koordináció

---

## 📚 **További Anyagok**

### `ZH_kezikonyv.md`
Teljes magyar nyelvű referencia:
- Ubuntu/ROS2 setup
- Build/run parancsok
- Node minták
- Üzenettípusok
- TF/quaternion használat

### `templates/`
Újrahasználható sablonok:
- `publisher_template.py`
- `subscriber_template.py`
- `params_node_template.py`
- `launch_xml_template.launch.xml`

---

## 🔍 Gyors Kereső Táblázat

| Téma | Gyakorlat |
|------|-----------|
| Publisher/Subscriber alapok | gyak2 |
| Path generálás | gyak3 |
| RViz markerek | gyak5 |
| LaserScan feldolgozás | gyak6, gyak9, gyak12 |
| Robot szimuláció | gyak7, gyak10 |
| PID szabályozás | gyak7 |
| TF transformációk | gyak8, gyak9, gyak11, gyak12 |
| Pure Pursuit | gyak10 |
| Fájl I/O | gyak11 |
| PointCloud2 | gyak6, gyak9, gyak12 |
| Multi-robot | gyak9 |
| Marker vizualizáció | gyak5, gyak6, gyak10, gyak12 |
| Paraméterek | gyak3, gyak7, gyak10, gyak11 |

---

**💡 Tipp:** Minden fájl magyar kommentekkel van ellátva, így offline is könnyen használható ZH-ra!
