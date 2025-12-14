# ROS2 Package Referencia Gyors Keresőhöz

Ez a dokumentum segít gyorsan megtalálni, melyik package-ből milyen kódrészleteket lehet átemelni.

---

## 📦 **gyak2** - Alapvető Publisher/Subscriber
**Kulcsszavak:** `create_publisher`, `create_subscriber`, `create_timer`, `timer_callback`, `String`, `logger`, alapvető node struktúra

**Mire jó:**
- Egyszerű publisher/subscriber példák
- Timer használat (periodikus callback)
- Logger használat (info/warn/error)
- Alapvető ROS2 node létrehozás és main() függvény

---

## 📦 **gyak3** - Path Generálás Odometriából
**Kulcsszavak:** `Odometry`, `Path`, `PoseStamped`, paraméterek (`declare_parameter`, `get_parameter`), lista méret korlát, header kezelés

**Mire jó:**
- Odometria fogadása és feldolgozás
- Path építése PoseStamped elemekből
- Paraméterek használata (topic név, max méret)
- Lista méret menedzselés (régi elemek törlése)
- Header frame_id és timestamp kezelés

---

## 📦 **gyak4** - C++ ROS2 (Üres CMake projekt)
**Kulcsszavak:** `CMakeLists.txt`, C++ ROS2, launch fájlok XML

**Mire jó:**
- C++ alapú ROS2 package struktúra
- CMake konfiguráció példa

---

## 📦 **gyak5** - Marker Vizualizáció és Bounding Box
**Kulcsszavak:** `Marker`, `MarkerArray`, `CUBE`, `SPHERE`, `Odometry`, `lifetime`, bounding box (x_min, x_max, y_min, y_max), path ritkítás

**Mire jó:**
- Vizualizációs markerek létrehozása (CUBE, SPHERE)
- MarkerArray használat (több marker egyidejűleg)
- Marker színezés (RGBA)
- Lifetime beállítás (marker élettartam)
- Bounding box számítás
- Path gyűjtés és ritkítás (30-as lépés)

---

## 📦 **gyak6** - LaserScan Feldolgozás és PointCloud
**Kulcsszavak:** `LaserScan`, `PointCloud2`, poláris → Descartes konverzió, `sensor_msgs_py.point_cloud2`, `SPHERE_LIST`, legközelebbi pont, range szűrés

**Mire jó:**
- LaserScan fogadása és feldolgozás
- Poláris koordináták (távolság, szög) → Descartes (x, y, z)
- PointCloud2 létrehozása
- Range validáció (range_min, range_max)
- Legközelebbi pont keresése
- MarkerArray SPHERE_LIST vizualizáció

---

## 📦 **gyak7** - PID Szabályozó és Differenciális Robot
**Kulcsszavak:** `PID`, `Kp`, `Ki`, `Kd`, `Twist`, `cmd_vel`, longitudinális/laterális szabályozás, `euler_from_quaternion`, szöghiba, differenciális kinematika, `quaternion_from_euler`, TF broadcast

**Mire jó:**
- PID szabályozó implementáció (Kp, Ki, Kd)
- Távolság és szöghiba számítás
- Twist üzenet (linear.x, angular.z) publikálás
- Quaternion → Euler konverzió (yaw)
- Differenciális robot szimulátor (x, y, yaw)
- Odometry és TF publikálás (odom → base_link)
- Robot kinematika (sebességből pozíció)

---

## 📦 **gyak8** - TF Transzformációk
**Kulcsszavak:** `TF Buffer`, `TF Listener`, `Static TF Broadcaster`, `can_transform`, `lookup_transform`, `TransformStamped`, koordináta-rendszer transzformáció

**Mire jó:**
- TF Buffer és Listener használat
- Transzformáció lekérdezés (lookup_transform)
- Transzformáció ellenőrzés (can_transform)
- Statikus transzformáció publikálás
- Koordináta-rendszer közötti konverzió
- Kerekek pozíciójának követése különböző frame-ekben

---

## 📦 **gyak9** - Több Robot LaserScan + TF Aggregálás
**Kulcsszavak:** `LaserScan`, több robot (namespace), `TF Buffer`, `do_transform_cloud`, `PointCloud2 merge`, globális frame, `tf2_sensor_msgs`, dictionary tárolás (frame_id szerint)

**Mire jó:**
- Több robot scan fogadása (namespace lista)
- LaserScan poláris → Descartes
- PointCloud2 TF transzformáció (do_transform_cloud)
- Több PointCloud összefűzése (merge)
- Dictionary használat (frame_id alapú tárolás)
- Globális koordináta-rendszerbe transzformálás

---

## 📦 **gyak10** - Ackermann Robot és Pure Pursuit
**Kulcsszavak:** `Ackermann kinematika`, `wheelbase`, `Pure Pursuit`, lookahead pont, legközelebbi pont keresés, `steering_angle`, `atan2`, `do_transform_pose_stamped`, Marker vizualizáció (legközelebbi/lookahead)

**Mire jó:**
- Ackermann robot szimulátor (wheelbase paraméter)
- Pure Pursuit path követési algoritmus
- Legközelebbi pont keresés (távolság minimalizálás)
- Lookahead pont számítás
- Kormányszög kalkuláció (steering_angle)
- TF használat: robot pozíció lekérdezés (map → base_link)
- PoseStamped TF transzformáció
- Marker vizualizáció (SPHERE, színek, lifetime)

---

## 📦 **gyak11** - Path Betöltés Fájlból
**Kulcsszavak:** fájl olvasás (text file), `Path`, `PoseStamped`, yaw → quaternion (`quaternion_from_euler`), min/max számítás, `MarkerArray` (x_min, y_min, x_max, y_max), publikálási frekvencia

**Mire jó:**
- Text fájl beolvasása (x, y, z, yaw)
- Path generálás fájlból
- Yaw (fok) → quaternion konverzió
- Min/max értékek keresése (x, y)
- Marker vizualizáció (piros gömbök min/max pontoknál)
- Publikálási frekvencia beállítás (publish_rate paraméter)

---

## 📦 **gyak12** - Távolság Számítás és LaserScan Centroid
**Kulcsszavak:** `Float64`, távolság számítás, feltételes publikálás (`math.floor`), súlyponti pont (centroid, átlag x/y/z), `PointCloud2`, Marker (SPHERE), TF transzformáció (scan → map)

**Mire jó:**
- Odometriából távolság számítás (Euklideszi távolság)
- Float64 üzenet használat
- Feltételes publikálás (floor érték változás)
- LaserScan → PointCloud2
- Súlyponti pont (centroid) számítás (átlag koordináták)
- Marker vizualizáció (zöld gömb)
- TF transzformáció (lokális → globális frame)
- PoseStamped publikálás (transzformált koordináták)

---

## 📦 **proba1** - Odometry + Path + Szögszámítás
**Kulcsszavak:** `Odometry`, `Path`, legközelebbi pont, yaw szög (`euler_from_quaternion`), `atan2`, szögkülönbség, `Float64`, `PoseStamped`

**Mire jó:**
- Odometry fogadás és yaw kiszámítás
- Path feldolgozás
- Legközelebbi pont keresés (gyak10-ből)
- Szög számítás (atan2) map x tengely relatív
- Szögkülönbség (target_yaw - current_yaw)
- Float64 publikálás

---

## 📦 **proba2** - Marker (Téglalap) + Timer + Header
**Kulcsszavak:** `Marker`, `CUBE`, koordináta csere (x ↔ y), orientáció másolás, `Header`, timer (0.2 Hz), timestamp publikálás, launch + RViz

**Mire jó:**
- Marker téglalap (CUBE) vizualizáció
- Marker méret beállítás (scale.x, y, z)
- Orientáció másolás (jármű irányába mutat)
- Koordináta csere (x és y felcserélés)
- Timer használat (0.2 Hz)
- Header timestamp publikálás
- Launch fájl (node + RViz)
- RViz config (Marker megjelenítés)

---

## 📦 **proba3** - LaserScan Szűrés + PoseArray
**Kulcsszavak:** `LaserScan`, szög szűrés (+/-5°), `Float64`, legközelebbi objektum, poláris → Descartes, `PoseArray`, `Pose`, launch + RViz (PoseArray vizualizáció)

**Mire jó:**
- LaserScan szög alapú szűrés (+/- tartomány)
- Range validáció (range_min, range_max)
- Legközelebbi objektum keresés
- Float64 publikálás (távolság)
- Poláris → Descartes konverzió (gyak9-ből)
- PoseArray létrehozás és publikálás
- Pose lista építés (position + orientation)
- Launch + RViz config (PoseArray megjelenítés nyilakkal)

---

## 📦 **zh** - MarkerArray Generálás + Legközelebbi Objektum
**Kulcsszavak:** `MarkerArray`, objektum lista (matematikai képlet), véletlen szín, gömb méret, timer (1 Hz), TF transzformáció (map → base_link), legközelebbi objektum

**Mire jó:**
- Matematikai képletből objektum generálás (1000 db)
- MarkerArray létrehozás ciklussal
- Véletlen színezés (random RGB)
- Gömb marker (15 cm átmérő)
- Timer (1000 ms)
- Legközelebbi objektum keresés (távolság minimum)
- TF transzformáció (globális → lokális frame)
- PoseStamped publikálás (transzformált koordináták)

---

## 📦 **zh_speed** - Két Odometry Távolság + Yaw Tömbben
**Kulcsszavak:** két `Odometry`, távolság számítás (két jármű között), `Float64`, `Float64MultiArray`, yaw számítás (`euler_from_quaternion`), timer (100 ms)

**Mire jó:**
- Két subscriber (agent1, agent2 odometry)
- Távolság számítás két pozíció között (sqrt)
- Float64 publikálás (távolság)
- Yaw számítás mindkét járműre
- Float64MultiArray (tömb) publikálás
- Timer (100 ms frekvencia)

---

## 📦 **zh_scan** - LaserScan Sáv Ellenőrzés + Bool
**Kulcsszavak:** `LaserScan`, sáv ellenőrzés (hossztengely, ±10 cm szélesség), maximális távolság (5 m), `Bool`, `MarkerArray` (visszaverődések gömbként), lateral deviation

**Mire jó:**
- LaserScan szűrés (sáv ellenőrzés)
- Lateral deviation számítás (oldalsó eltérés)
- Maximális távolság szűrés (5 m)
- Bool publikálás (objektum detektálás)
- MarkerArray vizualizáció (10 cm gömbök)
- Visszaverődések megjelenítése

---

## 🔍 **Gyors Keresési Segédlet**

### **Publisher/Subscriber alapok**
→ `gyak2`

### **Timer használat**
→ `gyak2`, `proba2`, `zh`, `zh_speed`

### **Odometry feldolgozás**
→ `gyak3`, `gyak7`, `gyak12`, `proba1`, `zh_speed`

### **Quaternion ↔ Euler (yaw)**
→ `gyak7`, `gyak10`, `gyak11`, `proba1`, `zh_speed`

### **Path kezelés**
→ `gyak3`, `gyak10`, `gyak11`, `proba1`

### **Marker vizualizáció**
→ `gyak5`, `gyak6`, `gyak10`, `gyak12`, `proba2`, `zh`, `zh_scan`

### **LaserScan feldolgozás**
→ `gyak6`, `gyak9`, `gyak12`, `proba3`, `zh_scan`

### **Poláris → Descartes konverzió**
→ `gyak6`, `gyak9`, `proba3`

### **PointCloud2**
→ `gyak6`, `gyak9`, `gyak12`

### **TF transzformációk**
→ `gyak7`, `gyak8`, `gyak9`, `gyak10`, `gyak12`, `zh`

### **Legközelebbi pont keresés**
→ `gyak6`, `gyak10`, `proba1`, `proba3`, `zh`

### **PID szabályozó**
→ `gyak7`

### **Robot szimulátor (kinematika)**
→ `gyak7` (differenciális), `gyak10` (Ackermann)

### **Pure Pursuit algoritmus**
→ `gyak10`

### **Float64 / Float64MultiArray**
→ `gyak12`, `proba1`, `proba3`, `zh_speed`

### **Bool publikálás**
→ `zh_scan`

### **PoseArray**
→ `proba3`

### **Fájl olvasás**
→ `gyak11`

### **Launch fájl + RViz**
→ `gyak10`, `gyak12`, `proba2`, `proba3`

### **Paraméterek (declare_parameter)**
→ `gyak3`, `gyak5`, `gyak7`, `gyak10`, `gyak11`

### **Header timestamp**
→ `gyak3`, `gyak9`, `proba2`, `proba3`

---

## 📝 **Használati Tippek**

1. **Alapvető node struktúra** → Kezdd a `gyak2`-vel
2. **Odometry + Path** → `gyak3`, `gyak10`
3. **LaserScan + vizualizáció** → `gyak6`, `gyak9`, `proba3`
4. **Marker típusok** → `gyak5` (CUBE, SPHERE), `proba2` (téglalap)
5. **TF használat** → `gyak8` (alap), `gyak9` (cloud transzformáció), `gyak10` (pose transzformáció)
6. **Távolság/szög számítás** → `gyak7`, `gyak10`, `gyak12`, `proba1`, `zh_speed`
7. **Szabályozás** → `gyak7` (PID), `gyak10` (Pure Pursuit)
8. **Robot modell** → `gyak7` (diff), `gyak10` (Ackermann)

---

**Készítve:** 2025-12-15  
**Forrás:** mgm_gyak workspace összes package dokumentációja
