#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import cv2
import pyvoronoi
from ros2py_voronoi.msg import Graph, GraphNode, GraphEdge
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
import matplotlib.pyplot as plt

class VoronoiDiagramCreator(Node):
    def __init__(self):
        super().__init__('voronoi_diagram_creator_node')
        map_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )        
        # Subscribe to the map
        self.sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, map_qos)
            
        # Publish the custom graph message
        self.pub = self.create_publisher(Graph, '/voronoi_graph', map_qos)
        
        self.get_logger().info("Voronoi Diagram Creator Node Started. Waiting for /map...")

    def map_callback(self, msg):
        self.get_logger().info("Map received! Vectorizing and computing Voronoi...")

        # 1. Extract map metadata
        width = msg.info.width
        height = msg.info.height
        res = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        # Convert 1D tuple to 2D numpy array
        grid = np.array(msg.data, dtype=np.int8).reshape((height, width))

        # 2. Thresholding: Create a binary image for OpenCV
        # 255 = Obstacle, 0 = Free Space
        binary_map = np.zeros((height, width), dtype=np.uint8)
        binary_map[(grid >= 50) | (grid == -1)] = 255

        # 3. OpenCV: Find contours of the walls
        contours, hierarchy = cv2.findContours(
            binary_map, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

        # 4. Initialize Pyvoronoi (Scaling factor 100 to handle float precision)
        pv = pyvoronoi.Pyvoronoi(10000)

        # Add the outer bounding box of the map to contain the diagram
        pv.AddSegment([[0, 0], [width, 0]])
        pv.AddSegment([[width, 0], [width, height]])
        pv.AddSegment([[width, height], [0, height]])
        pv.AddSegment([[0, height], [0, 0]])

        # 5. Simplify contours into line segments and feed to Pyvoronoi
        for cnt, hier in zip(contours, hierarchy[0]):
            # Douglas-Peucker line simplification. 
            # Epsilon 1.0 means we tolerate 1 pixel of deviation.
            approx = cv2.approxPolyDP(cnt, 1.0, closed=True)
            pts = [pt[0] for pt in approx]
            
            for i in range(len(pts)):
                p1 = pts[i]
                p2 = pts[(i + 1) % len(pts)]
                # Add the wall segment
                pv.AddSegment([[float(p1[0]), float(p1[1])], [float(p2[0]), float(p2[1])]])

        # 6. Compute the diagram!
        pv.Construct()

        # 7. Package the output into our custom ROS 2 message
        graph_msg = Graph()
        graph_msg.header = msg.header

        # Extract Nodes (Vertices)
        vertices = pv.GetVertices()
        for v in vertices:
            node_msg = GraphNode()
            # Convert grid coordinates back to real-world meters
            node_msg.x = origin_x + (v.X * res)
            node_msg.y = origin_y + (v.Y * res)
            graph_msg.nodes.append(node_msg)

        # Extract Edges
        edges = pv.GetEdges()
        for e in edges:
            # We only want finite edges that form the primary skeleton
            #if e.is_primary and e.is_linear:
                # Ensure the edge connects to valid vertex indices
                if e.start != -1 and e.end != -1:
                    edge_msg = GraphEdge()
                    edge_msg.start_index = e.start
                    edge_msg.end_index = e.end
                    graph_msg.edges.append(edge_msg)

        # 8. Publish the graph
        self.pub.publish(graph_msg)
        self.get_logger().info(f"Published graph with {len(graph_msg.nodes)} nodes and {len(graph_msg.edges)} edges.")
        # ==========================================
        # MATPLOTLIB DEBUG VISUALIZER
        # ==========================================
        plt.figure(figsize=(10, 10))
        
        # 1. Plot the raw OpenCV binary map
        # origin='lower' forces Y to go up, perfectly matching ROS map coordinates
        plt.imshow(binary_map, cmap='gray', origin='lower')

        # 2. Plot the vectorized OpenCV lines (Blue)
        for cnt, hier in zip(contours, hierarchy[0]):
            approx = cv2.approxPolyDP(cnt, 1.0, closed=True)
            pts = [pt[0] for pt in approx]
            for i in range(len(pts)):
                p1 = pts[i]
                p2 = pts[(i + 1) % len(pts)]
                # Plot line between X coords and Y coords
                plt.plot([p1[0], p2[0]], [p1[1], p2[1]], color='blue', linewidth=2)
        plt.figure()
        # 3. Plot the Pyvoronoi Graph Edges (Red)
        for e in edges:
            if e.start != -1 and e.end != -1:
                v1 = vertices[e.start]
                v2 = vertices[e.end]
                plt.plot([v1.X, v2.X], [v1.Y, v2.Y], color='red', linewidth=1)

        plt.title("Map Vectors (Blue) vs Voronoi Graph (Red)")
        plt.xlabel("X (Pixels)")
        plt.ylabel("Y (Pixels)")
        
        self.get_logger().info("Displaying Matplotlib plot. Close the window to continue...")
        plt.show() # WARNING: This freezes the node until you close the graph window!
def main(args=None):
    rclpy.init(args=args)
    node = VoronoiDiagramCreator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
