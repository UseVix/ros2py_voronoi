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
from scipy.ndimage import distance_transform_edt
from shapely.geometry import Polygon
from shapely.ops import unary_union
from shapely.geometry import Point as ShapelyPoint, LineString
from geometry_msgs.msg import Point as RosPoint
from shapely.validation import make_valid
from skimage.morphology import medial_axis
import sknw
import networkx as nx

# TODO Tune those below
# TODO Make those below setable by ros parameters
voronoi_distance_rejection_threshold_squared = 0 ** 2 # Meters?. Adjust based on your map resolution and noise level.
pyvoronoi_scaling_factor = 100000
vertex2vertex_merge_threshold_squared = 0.0 ** 2 # Meters?. Adjust based on your map resolution and noise level.
curved_edge_discretization_step = 0.1
douglas_pecker_epsilon = 50.0 # Pixels. Adjust based on your map resolution and desired simplification level.
hierarchy_level = 0
# TODO merge cameras somehow
import math

from skimage.draw import line

def has_line_of_sight(binary_grid, p1, p2):
    """
    Returns True if the straight line between p1 and p2 
    passes ONLY through navigable space (> 0).
    p1 and p2 are tuples of (row, col) or (y, x).
    """
    # Get the coordinates of all pixels along the straight line
    rr, cc = line(int(p1[0]), int(p1[1]), int(p2[0]), int(p2[1]))
    
    # Ensure we don't go out of bounds
    rr = np.clip(rr, 0, binary_grid.shape[0] - 1)
    cc = np.clip(cc, 0, binary_grid.shape[1] - 1)
    
    # Check if ANY pixel on the line is a wall (0)
    # If all pixels are free space (>0), it returns True
    return np.all(binary_grid[rr, cc] > 0)
def get_furthest_nodes(nx_graph):
    """Finds the two nodes separated by the longest path in the graph."""
    # Compute shortest paths between all pairs of nodes
    # (Since this is a skeleton graph, this is actually quite fast)
    path_lengths = dict(nx.all_pairs_dijkstra_path_length(nx_graph, weight='weight'))
    
    max_dist = 0
    node_a, node_b = None, None
    
    for source, targets in path_lengths.items():
        for target, dist in targets.items():
            if dist > max_dist:
                max_dist = dist
                node_a = source
                node_b = target
                
    return node_a, node_b
def pick_key_nodes(nx_graph, binary_grid):
    # 1. Pick the two furthest nodes
    n1, n2 = get_furthest_nodes(nx_graph)
    picked_nodes = [n1, n2]
    
    def get_coords(node_id):
        # Extracts (y, x) from the sknw graph
        return nx_graph.nodes[node_id]['pts'][0]
    
    while True:
        best_node = None
        max_distance_score = -1
        
        # 2. Find a new node that is as far away from the existing picked nodes as possible
        for candidate in nx_graph.nodes():
            if candidate in picked_nodes:
                continue
                
            c_coords = get_coords(candidate)
            
            # Simple heuristic: sum of distances to all currently picked nodes
            # (This forces the algorithm to pick a node in an unexplored corner)
            score = sum(math.dist(c_coords, get_coords(pn)) for pn in picked_nodes)
            
            if score > max_distance_score:
                max_distance_score = score
                best_node = candidate
                
        # 3. Check Line of Sight between the new best node and previously picked nodes
        best_coords = get_coords(best_node)
        los_found = False
        
        for pn in picked_nodes:
            if has_line_of_sight(binary_grid, best_coords, get_coords(pn)):
                los_found = True
                break
                
        # 4. Breaking Condition
        if los_found:
            # "cancel adding that node and carry on to pruning stage"
            print(f"Line of sight found! Stopping. Total key nodes: {len(picked_nodes)}")
            break
        else:
            # "don't cancel picking that node and pick next one"
            picked_nodes.append(best_node)
            
    return picked_nodes
def prune_to_shortest_paths(nx_graph, picked_nodes):
    """
    Creates a new graph containing ONLY the edges that make up 
    the shortest paths between the picked nodes.
    """
    edges_to_keep = set()
    
    # Find the shortest path between every combination of our picked nodes
    for j in range(1, len(picked_nodes)):
        source = picked_nodes[0]
        target = picked_nodes[j]
        
        try:
            # Get the sequence of nodes that form the shortest path
            path = nx.shortest_path(nx_graph, source=source, target=target, weight='weight')
            
            # Convert the sequence of nodes into a sequence of edges (A->B, B->C)
            for k in range(len(path) - 1):
                # Sort the tuple so (A,B) and (B,A) are treated identically
                edge = tuple(sorted((path[k], path[k+1])))
                edges_to_keep.add(edge)
        except nx.NetworkXNoPath:
            continue # Skip if no path exists (e.g., disconnected map)
                
    # Create a fresh graph and populate it with only the kept edges and nodes
    pruned_graph = nx.Graph()
    for u, v in edges_to_keep:
        # Copy the edge data (like the pixel coordinates of the line)
        edge_data = nx_graph.get_edge_data(u, v)
        pruned_graph.add_edge(u, v, **edge_data)
        
        # Copy the node data (like coordinates)
        pruned_graph.add_node(u, **nx_graph.nodes[u])
        pruned_graph.add_node(v, **nx_graph.nodes[v])
        
    return pruned_graph
class VoronoiDiagramCreator(Node):
    map_x = None
    map_y = None
    def __init__(self):
        super().__init__('voronoi_diagram_creator_node')
        map_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )        
        # Subscribe to the map with increased queue depth
        self.sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, map_qos)
            
        # Publish the custom graph message with larger queue
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
        
        if self.map_x is None or self.map_x.shape != grid.shape:
            unknown_locations_mask = (grid == -1)            
            indices = distance_transform_edt(
                unknown_locations_mask,
                return_distances=False,
                return_indices=True,
            )
            self.map_x = indices[1].astype(np.float32)
            self.map_y = indices[0].astype(np.float32)
        cv2.remap(grid, self.map_x, self.map_y, interpolation=cv2.INTER_NEAREST, dst=grid)
        binary_nav_grid = (grid <= 50)
        skeleton = medial_axis(binary_nav_grid)
        binary_nav_grid = binary_nav_grid.astype(np.uint8)
        nx_graph = sknw.build_sknw(skeleton)
        key_nodes = pick_key_nodes(nx_graph, binary_nav_grid)
        
        # 3. Prune the graph down to ONLY the paths connecting those key nodes
        final_pruned_graph = prune_to_shortest_paths(nx_graph, key_nodes)
        

        
        
        # 7. Package the output into our custom ROS 2 message
        graph_msg = Graph()
        graph_msg.header = msg.header
        # Maps sknw's arbitrary node IDs to your list index (0, 1, 2, ...)
        node_id_mapping = {}

        # --- Extract Nodes ---
        for i, node_id in enumerate(final_pruned_graph.nodes()):
            # sknw stores coordinates in standard image format: (row, column) which is (Y, X)
            pixel_y, pixel_x = final_pruned_graph.nodes[node_id]['pts'][0] 

            # Convert grid coordinates back to real-world ROS meters
            real_x = origin_x + (pixel_x * res)
            real_y = origin_y + (pixel_y * res)

            # Create and append your custom message node
            node_msg = GraphNode()
            node_msg.x = float(real_x)
            node_msg.y = float(real_y)
            graph_msg.nodes.append(node_msg)

            # Save mapping to connect the edges correctly
            node_id_mapping[node_id] = i

        # --- Extract Edges ---
        for start_id, end_id in final_pruned_graph.edges():
            # sknw also provides the exact pixel path for this edge in:
            # path_pixels = nx_graph[start_id][end_id]['pts']
            # You can use 'path_pixels' if you need to check if the edge hits an obstacle later!

            edge_msg = GraphEdge()
            # Map the sknw IDs to the indices in your graph_msg.nodes array
            edge_msg.start_index = node_id_mapping[start_id]
            edge_msg.end_index = node_id_mapping[end_id]
            
            graph_msg.edges.append(edge_msg)

        # 4. Publish!
        self.pub.publish(graph_msg)


        self.get_logger().info(f"Published graph with {len(graph_msg.nodes)} nodes and {len(graph_msg.edges)} edges.")
        """
        # ==========================================
        # MATPLOTLIB DEBUG VISUALIZER
        # ==========================================
        #plt.clf()
        plt.imshow(grid)
        plt.figure()
        plt.imshow(skeleton)
        plt.figure(figsize=(10, 10))
        
        # 1. Plot the raw OpenCV binary map
        # origin='lower' forces Y to go up, perfectly matching ROS map coordinates
        # plt.imshow(binary_map, cmap='gray', origin='lower')

        
        # 3. Plot the Pyvoronoi Graph Edges (Red)
        for e in graph_msg.edges:
            start_node = graph_msg.nodes[e.start_index]
            end_node = graph_msg.nodes[e.end_index]
            plt.plot([(start_node.x-origin_x)/res, (end_node.x-origin_x)/res], [(start_node.y-origin_y)/res, (end_node.y-origin_y)/res], color='red', linewidth=1)

        plt.title("Map Vectors (Blue) vs Voronoi Graph (Red)")
        plt.xlabel("X (Pixels)")
        plt.ylabel("Y (Pixels)")
        
        
        self.get_logger().info("Displaying Matplotlib plot. Close the window to continue...")
        plt.show() # WARNING: This freezes the node until you close the graph window!
        #plt.pause(0.01)
        """
        
def main(args=None):
    rclpy.init(args=args)
    #plt.ion()
    node = VoronoiDiagramCreator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
