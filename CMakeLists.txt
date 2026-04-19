cmake_minimum_required(VERSION 3.8)
project(ros2py_voronoi)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 1. Generate Custom Messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GraphNode.msg"
  "msg/GraphEdge.msg"
  "msg/Graph.msg"
  DEPENDENCIES std_msgs
)

# 2. Install the Python Script as an executable
install(PROGRAMS
  scripts/voronoi_diagram_creator_node.py
  scripts/graph_visualiser.py
  scripts/voronoi_saver.py
  scripts/voronoi_loader.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
