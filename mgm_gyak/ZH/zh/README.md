# zh ROS 2 csomag

Ez a skeleton a következő parancs eredménye lenne:
```
ros2 pkg create zh --build-type ament_python --dependencies \
  rclpy std_msgs geometry_msgs nav_msgs visualization_msgs tf2_ros
```
(Kiegészítve `tf_transformations` függőséggel, ha Euler–Quaternion konverzióra szükség lesz.)

## Fájlszerkezet
```
ZH/zh/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── zh
└── zh/
    └── __init__.py
```

## Build (Ubuntu / Humble)
```bash
source /opt/ros/humble/setup.bash
cd <workspace-gyökér>
colcon build --packages-select zh --symlink-install
source install/setup.bash
ros2 pkg list | grep '^zh$'
```

## Következő lépés ötletek
- Node hozzáadása: `zh_node.py` és entry point frissítés.
- Launch mappa és RViz config létrehozása.
- Paraméterezés, TF használat, Path vagy Marker publikálás.
