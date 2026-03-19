#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from ros2py_voronoi.msg import Graph
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

class VoronoiVisualizer(Node):
    def __init__(self):
        super().__init__('graph_visualiser')
        map_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        # Subscribe to your custom graph
        self.sub = self.create_subscription(Graph, '/voronoi_graph', self.graph_callback, map_qos)
        # Publish RViz markers
        self.pub = self.create_publisher(MarkerArray, '/voronoi_markers', map_qos)

    def graph_callback(self, msg):
        marker_array = MarkerArray()

        # 1. Create a Marker for the Edges (Lines)
        edge_marker = Marker()
        edge_marker.header = msg.header
        edge_marker.ns = "edges"
        edge_marker.id = 0
        edge_marker.type = Marker.LINE_LIST
        edge_marker.action = Marker.ADD
        edge_marker.scale.x = 0.02 # Line thickness (2cm)
        edge_marker.color.r = 0.0
        edge_marker.color.g = 1.0 # Green lines
        edge_marker.color.b = 0.0
        edge_marker.color.a = 1.0
        edge_marker.pose.orientation.w = 1.0

        # Draw lines using the start and end indices from your custom message
        for edge in msg.edges:
            start_node = msg.nodes[edge.start_index]
            end_node = msg.nodes[edge.end_index]
            
            p1 = Point(x=start_node.x, y=start_node.y, z=0.0)
            p2 = Point(x=end_node.x, y=end_node.y, z=0.0)
            
            edge_marker.points.append(p1)
            edge_marker.points.append(p2)

        # 2. Create a Marker for the Nodes (Dots)
        node_marker = Marker()
        node_marker.header = msg.header
        node_marker.ns = "nodes"
        node_marker.id = 1
        node_marker.type = Marker.POINTS
        node_marker.action = Marker.ADD
        node_marker.scale.x = 0.05 # Point size (5cm)
        node_marker.scale.y = 0.05
        node_marker.color.r = 1.0 # Red dots
        node_marker.color.g = 0.0
        node_marker.color.b = 0.0
        node_marker.color.a = 1.0
        node_marker.pose.orientation.w = 1.0

        for node in msg.nodes:
            node_marker.points.append(Point(x=node.x, y=node.y, z=0.05)) # Lift dots slightly above lines

        marker_array.markers.extend([edge_marker, node_marker])
        self.pub.publish(marker_array)
def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(VoronoiVisualizer())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
