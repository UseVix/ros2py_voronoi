Przed budową poza rosdep update trzeba zrobić pip install pyvoronoi <br>
1 Terminal:<br>
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/opt/ros/humble/share/nav2_bringup/maps/turtlebot3_world.yaml<br>
2 Terminal:<br>
ros2 run ros2py_voronoi voronoi_diagram_creator_node.py<br>
3 Terminal:<br>
ros2 run ros2py_voronoi graph_visualiser.py<br>
4 Terminal:<br>
ros2 run nav2_util lifecycle_bringup map_server && rviz2<br>
I można wziąć graf z topica /voronoi_graph
<img width="559" height="697" alt="image" src="https://github.com/user-attachments/assets/42a68681-1196-41e4-9379-c7ff98fd969a" />
