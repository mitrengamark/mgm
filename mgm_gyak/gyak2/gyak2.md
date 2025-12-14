# Gyak2 - ROS2 Publisher-Subscriber Alapok

## 📋 Csomag Áttekintése

A **gyak2** egy ROS2 alapúcsomagnak lett kialakítva, amely a **Publisher-Subscriber** kommunikációs minta bemutatására szolgál. Ez egy alapvető, de nagyon fontos ROS2 koncepció, amely lehetővé teszi, hogy több program (node) egymással kommunikáljon üzenetek (messages) küldésével.

### Mire jó ez a csomag?
- Megtanulsz, hogyan hozz létre egy **Publisher nodot** (amely üzeneteket küld)
- Megtanulsz, hogyan hozz létre egy **Subscriber nodot** (amely üzeneteket fogad)
- Megérted a ROS2 **Topic** alapú kommunikáció alapjait
- Megtanulsz a **callback függvényekkel** dolgozni

---

## 📁 Fájlstruktúra

```
gyak2/
├── gyak2/                    # Python package mappa
│   ├── __init__.py          # Üres, de Python csomagként ismeri fel
│   ├── publisher.py         # Publisher node implementációja
│   └── subscriber.py        # Subscriber node implementációja
├── test/                    # Tesztfájlok
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
├── resource/                # Erőforrások
├── package.xml              # ROS2 csomag metaadatai
├── setup.py                 # Python csomag konfigurációja
├── setup.cfg                # Python setup konfigurációja
└── README.md                # Ez a fájl
```

---

## 🔧 Csomag Konfigurációja (package.xml)

```xml
<?xml version="1.0"?>
<package format="3">
  <name>gyak2</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="doba.daniel@outlook.com">mgm</maintainer>
  <license>TODO: License declaration</license>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### Mi a teendő itt?
- **`<name>`**: A csomag neve - ezzel használod majd a `ros2 run` parancsban
- **`<test_depend>`**: Teszt eszközök, amelyek minőségi ellenőrzésre használódnak (kódstílus, docstring ellenőrzés)
- **`<build_type>ament_python</build_type>`**: Ez egy Python-alapú ROS2 csomag

---

## 📤 Publisher Node - publisher.py

### Mi az a Publisher?
A **Publisher** egy ROS2 node, amely **üzeneteket küld** egy ún. **Topic-ba**. Több node is hallgathatja ezeket az üzeneteket.

### Részletes magyarázat kódsor alapján:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
```
**Importok:**
- `rclpy`: ROS2 Python kliens könyvtár
- `Node`: az alap ROS2 node osztály
- `String`: egy egyszerű szöveg üzenettípus

---

```python
class Publisher_Node(Node):
    def __init__(self):
        super().__init__('publisher_node')
```
**Osztály definíció:**
- Egy új class-t hozunk létre, amely örökl a `Node` osztályból
- A `super().__init__()` meghívásával **regisztrálódik** az ROS rendszerben "publisher_node" navn alatt
- Ez az elnevezés látható lesz az ROS2 gráfban (`rclpy graph show`)

---

```python
self.publisher = self.create_publisher(String, "chatter", 10)
```
**Publisher létrehozása:**
- `String`: az üzenet típusa (szöveg)
- `"chatter"`: a **Topic** neve, ahová küldünk
- `10`: a QoS (Quality of Service) queue mérete - maximum 10 üzenet lehet az adatfolyamban

**Gyakorlati magyarázat:**
- Képzeld el, hogy létrehoztunk egy "csatornát" (chatter) ahol szöveges üzeneteket küldhetünk
- Ha a subscriber nem elég gyorsan olvassa az üzeneteket, akkor max. 10 üzenet tárolt fel a sorban

---

```python
self.timer = self.create_timer(1.0, self.timer_callback)
```
**Időzítő (Timer) létrehozása:**
- `1.0`: másodpercek (1 másodpercenként futtatódik)
- `self.timer_callback`: az a függvény, amely 1 másodpercenként meghívódik
- Ez a mód garantálja, hogy az üzenetek **szabályosan** küldetnek ki

---

```python
def timer_callback(self):
    message = "Hello, ROS2!"
    
    self.get_logger().info(message)
    self.get_logger().warn(message)
    self.get_logger().error(message)
```
**A callback függvény:**
- Ezt **1 másodpercenként** meghívja az ROS rendszer
- `self.get_logger()`: naplózási eszköz, amellyel követheted, mi történik
  - `.info()`: normál információ (kék szín)
  - `.warn()`: figyelmeztetés (sárga szín)
  - `.error()`: hiba (piros szín)

---

```python
msg = String()
msg.data = message

self.publisher.publish(msg)
```
**Üzenet küldése (publikálás):**
1. Létrehozunk egy `String` típusú üzenet objektumot
2. Kitöltjük a `.data` mezőt a szövegünkkel
3. A `publish()` metódussal ténylegesen kiküldük a "chatter" topicba

---

```python
def main(args=None):
    rclpy.init(args=args)
    
    publisher_node = Publisher_Node()
    
    rclpy.spin(publisher_node)
    
    rclpy.shutdown()
```
**Az alkalmazás indítása:**
- `rclpy.init()`: inicializálja a ROS2 rendszert
- `Publisher_Node()`: létrehozza a node-ot
- `rclpy.spin()`: **eseményciklus** - a node futni kezd és hallgatja a timereket
  - Ez egy végtelen ciklus, amely csak Ctrl+C-vel szakítható meg
- `rclpy.shutdown()`: tiszta leállítás

---

## 📥 Subscriber Node - subscriber.py

### Mi az a Subscriber?
A **Subscriber** egy ROS2 node, amely **hallgatja** a topicot és **üzeneteket fogad**.

### Jelenlegi állapot (HIÁNYOS!)

```python
class Subscriber_Node(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        
        self.subscription = self.create_subscription()
```

⚠️ **PROBLÉMA:** A `create_subscription()` param nélkül van meghívva!

### Helyes forma - mit kellene változtatni:

```python
self.subscription = self.create_subscription(
    String,                      # Üzenettípus
    'chatter',                   # Topic név (UGYAN AZ, mint a publishernél!)
    self.listener_callback,      # Callback függvény
    10                           # QoS mélység
)
```

### Mit kellene implementálni:

```python
def listener_callback(self, msg):
    """Ez a függvény meghívódik, amikor új üzenet érkezik"""
    self.get_logger().info(f'Kapott üzenet: {msg.data}')
```

**Magyarázat:**
- A callback függvény **automatikusan meghívódik**, amikor a "chatter" topicra új üzenet érkezik
- `msg` paraméter tartalmazza a kapott üzenetet
- `msg.data` a szöveg mező a String üzenetből

---

## 🎯 Működés - Teljes Folyamat

```
Publisher Node (publisher.py)
    ↓
    ├─→ Timer: 1 másodpercenként
    ├─→ Létrehoz egy "Hello, ROS2!" üzenetet
    └─→ Publikus-ál a "chatter" topicba
        ↓
    
    ROS2 Middleware (közvetítő)
        ↓
        
Subscriber Node (subscriber.py)
    ├─→ Hallgatja a "chatter" topicot
    ├─→ Új üzenet érkezik
    └─→ Meghívja a listener_callback()-et
        └─→ Kiírja: "Kapott üzenet: Hello, ROS2!"
```

### Idővonal (egy 3 másodperces futás):
```
0.0s: Publisher node kezd futni
1.0s: Timer tűz → publish "Hello, ROS2!" → Subscriber fogad → log: "Kapott üzenet: Hello, ROS2!"
2.0s: Timer tűz → publish "Hello, ROS2!" → Subscriber fogad → log: "Kapott üzenet: Hello, ROS2!"
3.0s: Timer tűz → publish "Hello, ROS2!" → Subscriber fogad → log: "Kapott üzenet: Hello, ROS2!"
...
```

---

## 🚀 Hogyan Futtasd

### Előfeltételek:
```bash
# ROS2 telepítve van
# A csomag buildelt: colcon build --packages-select gyak2
```

### 1. Terminal - Publisher indítása:
```bash
source install/setup.bash
ros2 run gyak2 publisher
```

Kimenete:
```
[INFO] [publisher_node]: Hello, ROS2!
[WARN] [publisher_node]: Hello, ROS2!
[ERROR] [publisher_node]: Hello, ROS2!
```

### 2. Másik Terminal - Subscriber indítása:
```bash
source install/setup.bash
ros2 run gyak2 subscriber
```

**Ha jól működik, lát ugyanazokat az üzeneteket!**

### Diagnosztika - Mi történik a háttérben?
```bash
# A topic lista:
ros2 topic list

# A "chatter" topic részletei:
ros2 topic info /chatter

# Az üzenetek valós időben nézegethető:
ros2 topic echo /chatter
```

---

## 📚 Tanulási Tippek

### 1. **Megértsd a Topic fogalmát**
   - A topic = egy névvel ellátott kommunikációs csatorna
   - Bármelyik node publikálhat ugyanarra
   - Bármelyik node hallgathatja

### 2. **QoS = Quality of Service**
   - `10` azt jelenti: max 10 üzenet a sorban
   - Nagyobb szám = több memória, de kevésbé veszítünk el üzenetet
   - Valós robotok esetén kritikus!

### 3. **Callback függvények**
   - `timer_callback`: időzítő tüzelése
   - `listener_callback`: üzenet érkezése
   - Ezeket az ROS2 **automatikusan** hívja meg - nem te!

### 4. **Logging szintek**
   - `info()`: normál információ
   - `warn()`: figyelmeztetés
   - `error()`: hiba
   - `debug()`: hibakereséshez
   - **Könyvben használd az `info()` szintet leginkább!**

### 5. **ROS2 Gráf Vizualizáció**
   ```bash
   rqt_graph
   ```
   Ezzel vizuálisan láthatsz minden node-ot és topic-ot!

---

## 🔍 Gyakori Hibák és Megoldások

| Hiba | Oka | Megoldás |
|------|-----|----------|
| `ModuleNotFoundError: No module named 'rclpy'` | ROS2 nem inicializálva | `source install/setup.bash` futtatása |
| Subscriber nem kap üzeneteket | A topic nevek nem egyeznek | Ellenőrizd: Publisher "chatter" → Subscriber "chatter" |
| "publisher_node" gráfban nem látható | Node még nem futott meg | Futtatd az `ros2 run` parancsot |
| Sok üzenet elvész | QoS túl kicsi | Növeld a számot 10-ből 50-re vagy többre |

---

## 💡 Vizsgára Felkészülés

### Amit tudnod kell:
- ✅ Mi a Publisher és Subscriber
- ✅ Hogyan hozz létre köztük Topic-ot
- ✅ Hogy működik a timer callback
- ✅ Hogy működik az üzenet callback
- ✅ Mi a QoS és miért fontos
- ✅ Hogyan loggol az ROS2

### Gyakorlási ötletek:
1. **Módosítsd** az üzenet szövegét - pl. dátum/idő kiírása
2. **Változtasd** a timer időjét - gyorsítsd fel (0.5s), vagy lassítsd (2.0s)
3. **Keress fel több adatot** - pl. számláló, amely nő minden üzenettel
4. **Adj hozzá több topicot** - egy publisher több topicra publikálhat
5. **Tedd komplexabbé az üzeneteket** - nem csak String, hanem custom message type

---

## 📖 Hasznos ROS2 Parancsok

```bash
# Fut-e valami?
ros2 node list

# Milyen topicok vannak?
ros2 topic list

# Mi a topicban?
ros2 topic echo /chatter

# Ki publikus-ál a topicra?
ros2 topic info /chatter

# Teljes ROS gráf:
ros2 graph show

# Vizuális gráf:
rqt_graph

# Log naplózás (csak az utolsó 50 sor):
ros2 launch gyak2 ... | tail -50
```

---

## 🎓 Összefoglalás

A **gyak2** csomag az alapvető ROS2 kommunikációt mutatja be. A Publisher-Subscriber minta az összes ROS2 alkalmazás alapja. Miután ezt megérted:

1. Készen állsz a bonyolultabb message típusokra
2. Megérted, hogyan épülnek fel a ROS2 robotok
3. Képes leszel saját sensor/aktor integrációkra

**Boldog tanulást!** 🚀

