Przed budową poza rosdep update trzeba zrobić pip install pyvoronoi <br>
1 Terminal:<br>
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/opt/ros/humble/share/nav2_bringup/maps/turtlebot3_world.yaml<br>
2 Terminal:<br>
ros2 run ros2py_voronoi voronoi_diagram_creator_node.py<br>
3 Terminal:<br>
ros2 run ros2py_voronoi graph_visualiser.py<br>
4 Terminal:<br>
ros2 run nav2_util lifecycle_bringup map_server && rviz2<br>
TODO:
odfiltrować z finalnego grafu węzły i krawędzie które leżą na nieznanych/zajętych terenach. Najlepiej zrobić to z pomocą shapeley to też pewnie się troszkę zmieni w zależności od tego jak będą wyglądały occupancy mapy
Dodać kategoryzacje lewo/prawo/prosto
