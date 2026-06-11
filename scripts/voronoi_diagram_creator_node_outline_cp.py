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
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point as RosPoint
from shapely.validation import make_valid
def ensure_2d_polygons(geometry):
    """Strips out lines and points, keeping only 2D polygon structures."""
    if geometry.is_empty:
        return geometry
    if geometry.geom_type in ['Polygon', 'MultiPolygon']:
        return geometry
    if geometry.geom_type == 'GeometryCollection':
        # Extract only the 2D polygon components from the collection
        polys = [g for g in geometry.geoms if g.geom_type in ['Polygon', 'MultiPolygon']]
        return unary_union(polys)
    # If the geometry is entirely a LineString or Point, return an empty Polygon
    return Polygon()
# TODO Tune those below
# TODO Make those below setable by ros parameters
voronoi_distance_rejection_threshold_squared = 0 ** 2 # Meters?. Adjust based on your map resolution and noise level.
pyvoronoi_scaling_factor = 100000
vertex2vertex_merge_threshold_squared = 0.0 ** 2 # Meters?. Adjust based on your map resolution and noise level.
curved_edge_discretization_step = 0.1
douglas_pecker_epsilon = 50.0 # Pixels. Adjust based on your map resolution and desired simplification level.
hierarchy_level = 0
# TODO merge cameras somehow
# TODO add  voronoi input visualisation in rviz (publish the simplified contours as a MarkerArray)
class VoronoiDiagramCreator(Node):
    map_x = None
    map_y = None
    def __init__(self):
        super().__init__('voronoi_diagram_creator_node')
        map_qos = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )        
        # Subscribe to the map with increased queue depth
        self.sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, map_qos)
            
        # Publish the custom graph message with larger queue
        self.pub = self.create_publisher(Graph, '/voronoi_graph', map_qos)
        # Publish simplified contours for RViz visualization with larger queue
        self.contour_pub = self.create_publisher(MarkerArray, '/voronoi_input_markers', map_qos)
        
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

        binary_map = np.zeros((height, width), dtype=np.uint8)
        binary_map[(grid <= 50)] = 255
        plt.imshow(binary_map)
        cv2.remap(binary_map, self.map_x, self.map_y, interpolation=cv2.INTER_NEAREST, dst=binary_map)
        plt.figure()
        plt.imshow(binary_map)
        # 3. OpenCV: Find contours of the walls
        contours, hierarchy = cv2.findContours(
            binary_map, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # 4. Initialize Pyvoronoi (Scaling factor 100 to handle float precision)
        pv = pyvoronoi.Pyvoronoi(pyvoronoi_scaling_factor)

        # Add the outer bounding box of the map to contain the diagram
        #pv.AddSegment([[0, 0], [width, 0]])
        #pv.AddSegment([[width, 0], [width, height]])
        #pv.AddSegment([[width, height], [0, height]])
        #pv.AddSegment([[0, height], [0, 0]])
        holes = []
        outers = []
        
        contour_markers = MarkerArray()
        contour_marker = Marker()
        contour_marker.header = msg.header
        contour_marker.ns = "simplified_contours"
        contour_marker.id = 0
        contour_marker.type = Marker.LINE_LIST
        contour_marker.action = Marker.ADD
        contour_marker.scale.x = 0.03
        contour_marker.color.r = 0.1
        contour_marker.color.g = 0.7
        contour_marker.color.b = 1.0
        contour_marker.color.a = 1.0
        contour_marker.pose.orientation.w = 1.0
        
        # 5. Simplify contours into line segments and feed to Pyvoronoi
        for cnt, hier in zip(contours, hierarchy[0]):
            # Douglas-Peucker line simplification. 
            # Epsilon 1.0 means we tolerate 1 pixel of deviation.
            parent_idx = hier[3] 
                    # Calculate depth by climbing the family tree until we hit the top (-1)
            level = 0
            while parent_idx != -1:
                level += 1
                parent_idx = hierarchy[0][parent_idx][3] # Move up to the next parent
            approx = cv2.approxPolyDP(cnt, douglas_pecker_epsilon, closed=True)
            pts = [pt[0] for pt in approx]
            if level==hierarchy_level+1 and len(pts) >= 3:
                holes.append(make_valid(Polygon(pts)))
            if level==hierarchy_level and len(pts) >= 3:
                outers.append(make_valid(Polygon(pts)))

            

            for i in range(len(pts)):
                p1 = pts[i]
                p2 = pts[(i + 1) % len(pts)]
                # Add the wall segment
                pv.AddSegment([[float(p1[0]), float(p1[1])], [float(p2[0]), float(p2[1])]])
                
                contour_marker.points.append(RosPoint(
                    x=origin_x + (float(p1[0]) * res),
                    y=origin_y + (float(p1[1]) * res),
                    z=0.01,
                ))
                contour_marker.points.append(RosPoint(
                    x=origin_x + (float(p2[0]) * res),
                    y=origin_y + (float(p2[1]) * res),
                    z=0.01,
                ))
            contour_markers.markers.append(contour_marker)

        # 6. Compute the diagram!
        pv.Construct()

        # 7. Package the output into our custom ROS 2 message
        graph_msg = Graph()
        graph_msg.header = msg.header
        # Let's pretend you have these two raw lists of Shapely Polygons


        # 1. Merge all outer boundaries into a single Master Geometry
        # (This handles overlapping rooms perfectly, too!)
        master_outers = unary_union(outers)

        # 2. Merge all hole boundaries into a single Master Geometry
        master_holes = unary_union(holes)
        
        
        master_outers = ensure_2d_polygons(master_outers)
        master_holes = ensure_2d_polygons(master_holes)
        # 3. Punch the holes out of the outers in one C++ optimized step!
        final_navigable_map = master_outers.difference(master_holes, grid_size=0.001)

        # Done! 'final_navigable_map' is now a mathematically perfect 
        # MultiPolygon with all holes correctly nested inside their proper parents.
        # Extract Nodes (Vertices)
        vertices = pv.GetVertices()
        vertex_degree = [0] * len(vertices)
        edges = pv.GetEdges()
        new_indices=[]
        Cells = pv.GetCells()
        removed_edges = []
        for i,e in enumerate(edges):
            c = Cells[e.cell]
            if c.contains_point:
                point=pv.RetrieveScaledPoint(c)
                point = pyvoronoi.Vertex(point[0], point[1])
            else:
                point=pyvoronoi.Vertex(*pv.RetrieveScaledSegment(c)[0])
            distance_squared=(point.X-vertices[e.start].X)**2+(point.Y-vertices[e.start].Y)**2
            if distance_squared > voronoi_distance_rejection_threshold_squared:
                vertex_degree[e.start] += 1
                vertex_degree[e.end] += 1
            else:
                removed_edges.append(i)
        for i,v in enumerate(vertices):
            merged = False
            for j,vn in enumerate(new_indices):
                if vn != -1 and (vertices[j].X-v.X)**2+(vertices[j].Y-v.Y)**2 < vertex2vertex_merge_threshold_squared:
                    new_indices.append(vn)
                    merged = True
                    break
            
            if merged:
                continue

            if final_navigable_map.contains(ShapelyPoint(v.X,v.Y)) and vertex_degree[i] > 0:
                node_msg = GraphNode()
                # Convert grid coordinates back to real-world meters
                node_msg.x = origin_x + (v.X * res)
                node_msg.y = origin_y + (v.Y * res)
                graph_msg.nodes.append(node_msg)
                new_indices.append(len(graph_msg.nodes)-1)
            else:
                new_indices.append(-1)

        # Extract Edges
        
        for i,e in enumerate(edges):
            # We only want finite edges that form the primary skeleton
            if i not in removed_edges:
                if e.is_linear:
                    # Ensure the edge connects to valid vertex indices
                    start=vertices[e.start]
                    end=vertices[e.end]
                    line=LineString([(start.X,start.Y),(end.X,end.Y)])
                    new_start = new_indices[e.start]
                    new_end = new_indices[e.end]
                    if e.start != -1 and e.end != -1 and line.within(final_navigable_map) and new_start != -1 and new_end != -1:
                        edge_msg = GraphEdge()
                        edge_msg.start_index = new_start
                        edge_msg.end_index = new_end
                        graph_msg.edges.append(edge_msg)
                else:
                    pindex = None
                    pvertex = None
                    try:
                        for v in pv.DiscretizeCurvedEdge(i,curved_edge_discretization_step):
                            if final_navigable_map.contains(ShapelyPoint(v[0],v[1])):
                                node_msg = GraphNode()
                                # Convert grid coordinates back to real-world meters
                                node_msg.x = origin_x + (v[0] * res)
                                node_msg.y = origin_y + (v[1] * res)
                                graph_msg.nodes.append(node_msg)
                                index = len(graph_msg.nodes)-1
                            else:
                                index = -1
                            new_indices.append(-1)
                            new_indices.append(index)
                            if pindex != -1 and index != -1 and pindex is not None:
                                line = LineString([pvertex, v])
                                if line.within(final_navigable_map):
                                    edge_msg = GraphEdge()
                                    edge_msg.start_index = pindex
                                    edge_msg.end_index = index
                                    graph_msg.edges.append(edge_msg)
                            pindex = index
                            pvertex = v
                    except ZeroDivisionError as e:
                        self.get_logger().debug(f"Skipping curved edge {i} due to pyvoronoi numeric instability: {e}")
                    except Exception as e:
                        self.get_logger().warn(f"Skipping curved edge {i} due to error: {e}")

                # Publish the simplified contours for RViz
                
        
        contour_markers.markers.append(contour_marker)
        self.contour_pub.publish(contour_markers)
        # 8. Publish the graph
        self.pub.publish(graph_msg)
        self.get_logger().info(f"Published graph with {len(graph_msg.nodes)} nodes and {len(graph_msg.edges)} edges.")
        # ==========================================
        # MATPLOTLIB DEBUG VISUALIZER
        # ==========================================
        
        plt.figure(figsize=(10, 10))
        
        # 1. Plot the raw OpenCV binary map
        # origin='lower' forces Y to go up, perfectly matching ROS map coordinates
        # plt.imshow(binary_map, cmap='gray', origin='lower')

        # 2. Plot the vectorized OpenCV lines (Blue)
        for cnt, hier in zip(contours, hierarchy[0]):
            approx = cv2.approxPolyDP(cnt, douglas_pecker_epsilon, closed=True)
            pts = [pt[0] for pt in approx]
            for i in range(len(pts)):
                p1 = pts[i]
                p2 = pts[(i + 1) % len(pts)]
                # Plot line between X coords and Y coords
                plt.plot([p1[0], p2[0]], [p1[1], p2[1]], color='blue', linewidth=2)
        
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
        
def main(args=None):
    rclpy.init(args=args)
    node = VoronoiDiagramCreator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
