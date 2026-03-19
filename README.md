Przed budową poza rosdep update trzeba zrobić pip install pyvoronoi
1 Terminal:
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/opt/ros/humble/share/nav2_bringup/maps/turtlebot3_world.yaml
2 Terminal:
ros2 run ros2py_voronoi voronoi_diagram_creator_node.py
3 Terminal:
ros2 run ros2py_voronoi graph_visualiser.py
4 Terminal:
ros2 run nav2_util lifecycle_bringup map_server && rviz2
