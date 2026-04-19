# ros2py_voronoi

Projekt generuje globalny graf Voronoi na podstawie mapy `nav_msgs/OccupancyGrid`, wizualizuje go w RViz oraz zapisuje do pliku JSON z klasyfikacją skrzyżowań.

---

## Instalacja zależności

### ROS2 Humble + Nav2 + Python
```bash
sudo apt update
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup
pip install pyvoronoi shapely
```

## Budowa pakietu
```bash
rosdep update
colcon build --symlink-install
source install/setup.bash
```
# Uruchamianie
Projekt wymaga 5 - 6 okien terminala
## 1.Map server (Nav2)
```bash
ros2 run nav2_map_server map_server \
  --ros-args -p yaml_filename:=/opt/ros/humble/share/nav2_bringup/maps/turtlebot3_world.yaml
```
## 2.Generator grafu Voronoi
```bash
ros2 run ros2py_voronoi voronoi_diagram_creator_node.py
```
Publikuje:
- /voronoi_graph
- QoS: Transient Local
## 3.Wizualizacja grafu
```bash
ros2 run ros2py_voronoi voronoi_diagram_creator_node.py
```
Publikuje:
- /voronoi_markers
- QoS: Transient Local

## 4.Dummy map
Do poprawnego wyświetlania się grafu należy uruchomić dummy map
```bash
ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id base_link
```

## 5. RViz + TF
```bash
ros2 run nav2_util lifecycle_bringup map_server && rviz2
```
Do poprawnego działania należy:
- Dodać Map i Voronoi markers wchodząc w Add -> By Topic -> Map / /Voronoi_Markers
- Wejść w displays Map i /voronoi markers -> Topic -> Durability policy -> Transient Local

## 6. Zapis grafu do json
```
ros2 run ros2py_voronoi voronoi_saver.py
```
Tworzy plik: voronoi_graph.json
### Struktura .json
```json
{
  "nodes": {
    "0": {"x": 1.23, "y": 4.56},
    "1": {"x": 1.40, "y": 4.80}
  },
  "edges": [
    {"from": 0, "to": 1},
    {"from": 1, "to": 5}
  ],
  "intersections": {
    "12": {"type": "T", "degree": 3},
    "27": {"type": "X", "degree": 4}
  },
  "dead_ends": {
    "3": {"x": 2.1, "y": 5.4, "degree": 1}
  },
  "inter_err": {
    "9": {"x": 3.2, "y": 6.1, "degree": 2}
  }
}
```
### Opis pól

- **nodes** – wszystkie węzły grafu (pozycje)
- **edges** – połączenia między węzłami
- **intersections** – skrzyżowania:
  - `T` (degree = 3)
  - `X` (degree = 4)
  - `complex` (degree > 4)
- **dead_ends** – węzły o degree = 1
- **inter_err** – błędne skrzyżowania (degree = 2)

## Wizualizacja JSON offline
```bash
python3 scripts/plot_graph.py
```

<img width="559" height="697" alt="image" src="https://github.com/user-attachments/assets/42a68681-1196-41e4-9379-c7ff98fd969a" />
