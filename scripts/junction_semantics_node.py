#!/usr/bin/env python3

from math import atan2
import math
import re
import rclpy
from rclpy.node import Node
import numpy as np
import pyvoronoi
from ros2py_voronoi.msg import Graph, GraphNode, GraphEdge, JunctionData, PolarCoordinates
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import String

class PolarCoordinatesCOnverter(Node):
    def __init__(self):
        super().__init__('voronoi_diagram_creator_node')
        map_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )        
    
        self.graph_sub = self.create_subscription(
            Graph, '/voronoi_graph', self.graph_callback, map_qos)
        self.localisation_sub = self.create_subscription(
            localisationtype, '/localisation', self.localisation_callback, map_qos)
        self.commands_sub = self.create_subscription(
            String, '/commands', self.commands_callback, map_qos)
        self.pub = self.create_publisher(JunctionData, '/junction_data', map_qos)

        self.get_logger().info("Junction Semantics Node Started. Waiting for /voronoi_graph and /localisation...")
        self.latest_localisation = None
        self.latest_graph = None
        self.commands = []
        self.previous_junction_node = None
        self.command_is_being_processed = False
    def commands_callback(self, msg):
        self.get_logger().info(f"Received command: {msg.data}")
        self.commands.append(msg.data)
    def distance_point_to_line(self, p1, p2,x3,y3):
        """
        p1: (x1, y1) - pierwszy punkt na prostej
        p2: (x2, y2) - drugi punkt na prostej
        p3: (x3, y3) - punkt poza prostą (badany)
        """
        x1, y1 = p1.x, p1.y
        x2, y2 = p2.x, p2.y
        
        # Wektor kierunkowy prostej (od p1 do p2)
        dx = x2 - x1
        dy = y2 - y1
        
        # Warunek bezpieczeństwa: jeśli p1 i p2 to ten sam punkt
        if dx == 0 and dy == 0:
            return float('inf')
            
        # Obliczenie współczynnika rzutu u (pozycja punktu Q na prostej względem p1 i p2)
        # Jest to iloczyn skalarny wektorów (p3-p1) oraz (p2-p1) podzielony przez kwadrat długości (p2-p1)
        u = ((x3 - x1) * dx + (y3 - y1) * dy) / (dx**2 + dy**2)
        
        # u określa położenie punktu Q:
        # u = 0 oznacza punkt p1
        # u = 1 oznacza punkt p2
        # 0 <= u <= 1 oznacza, że punkt leży MIĘDZY punktami p1 i p2
        if u < 0 or u > 1:
            return float('inf')
            
        # Wyznaczenie współrzędnych najbliższego punktu Q na prostej
        qx = x1 + u * dx
        qy = y1 + u * dy
        
        # Obliczenie odległości Euklidesowej między punktem p3 a rzutem Q
        odleglosc = math.sqrt((x3 - qx)**2 + (y3 - qy)**2)
        
        return odleglosc
    def normalise_angle(self, angle):
        return angle % -math.copysign(math.pi, angle)
    def get_junction(self, graph, car_coordinates):
        car_orientation = car_coordinates.orientation
        car_x = car_coordinates.x
        car_y = car_coordinates.y
        
        closest_edge = None
        closest_edge_distance = float('inf')
        for edge in graph.edges:
            start_node = graph.nodes[edge.start_index]
            end_node = graph.nodes[edge.end_index]
            # Check if the car is close to the edge (within a certain threshold)
            distance_to_edge = self.distance_point_to_line(start_node, end_node, car_x, car_y)
            if distance_to_edge < closest_edge_distance:
                closest_edge_distance = distance_to_edge
                closest_edge = edge
        if closest_edge is not None:
            start_node = graph.nodes[closest_edge.start_index]
            end_node = graph.nodes[closest_edge.end_index]
            edge1_angular_distance = abs(self.normalise_angle(car_orientation - atan2(start_node.y - car_y, start_node.x - car_x)))
            edge2_angular_distance = abs(self.normalise_angle(car_orientation - atan2(end_node.y - car_y, end_node.x - car_x)))
            if edge1_angular_distance < edge2_angular_distance:
                junction_node = start_node
                visited_node = end_node
                
            else:            
                junction_node = end_node
                visited_node = start_node
            angles = []
            nodes = []
            reference_angle = atan2(visited_node.y - junction_node.y, visited_node.x - junction_node.x)
            for edge in graph.edges:
                potential_destination_node = None

                if (graph.nodes[edge.end_index] == junction_node):
                    potential_destination_node = graph.nodes[edge.end_index]
                if (graph.nodes[edge.start_index] == junction_node):
                    potential_destination_node = graph.nodes[edge.start_index]
                if potential_destination_node is not None and potential_destination_node != visited_node:
                        angles.append(self.normalise_angle(atan2(potential_destination_node.y - junction_node.y, potential_destination_node.x - junction_node.x)-reference_angle))
                        nodes.append(potential_destination_node)
            pairs = sorted(zip(angles, nodes), key=lambda p: p[0])
            angles, nodes = map(list, zip(*pairs))
            match (len(nodes),len(self.commands)):
                case 0,_:
                    destination_node = GraphNode(x=float('nan'), y=float('nan'))
                case (node_count, command_count) if node_count == 1 or command_count == 0:
                    destination_node = nodes[0]
                case (node_count, command_count) if node_count > 1 and command_count > 0:
                    self.command_is_being_processed = True
                    m = re.search(r"\d+", self.commands[0])
                    number = int(self.commands[0][m.start():]) if m else 0
                    direction = self.commands[0][:m.start()] if m else self.commands[0]
                    match direction.lower():
                        case "left" or "l" or "lewo" or "lewy":
                            destination_node = nodes[number-1]
                        case "right" or "r" or "prawo" or "prawy":
                            destination_node = nodes[-number]
                        case "straight" or "s" or "forward" or "f" or "prosto" or "prosty":
                            destination_node = nodes[np.argmin(np.abs(angles))] 
        else:
            visited_node = GraphNode(x=float('nan'), y=float('nan'))
            junction_node = GraphNode(x=float('nan'), y=float('nan'))
            destination_node = GraphNode(x=float('nan'), y=float('nan'))
        return visited_node, junction_node, destination_node
    def GeneratePolarCoordinates(self,car_coordinates,junction_coordinates):
        car_x = car_coordinates.x
        car_y = car_coordinates.y
        car_orientation = car_coordinates.orientation
        junction_x = junction_coordinates.x
        junction_y = junction_coordinates.y
        # Calculate the angle from the car to the junction
        angle_to_junction = atan2(junction_y - car_y, junction_x - car_x)
        # Calculate the distance from the car to the junction
        distance_to_junction = math.sqrt((junction_x - car_x)**2 + (junction_y - car_y)**2)
        # Calculate the relative angle (difference between car's orientation and angle to junction)
        relative_angle = angle_to_junction - car_orientation
        return PolarCoordinates(theta=relative_angle, r=distance_to_junction)
    def graph_callback(self, msg):
        self.get_logger().info("Received /voronoi_graph message. Waiting for /localisation data...")
        self.latest_graph = msg
        self.callback(self.latest_graph, self.latest_localisation)
    def localisation_callback(self, msg):
        self.get_logger().info("Received /localisation message. Waiting for /voronoi_graph data...")
        self.latest_localisation = msg
        self.callback(self.latest_graph, self.latest_localisation)
    def callback(self, graph, localisation):
        if graph is None or localisation is None or len(self.commands) == 0:
            return  # Wait until both messages are received
                # Publish the junction semantics data
        junction_data_msg = JunctionData()
        junction_data_msg.header.stamp = self.get_clock().now().to_msg()
        junction_data_msg.visited_node, junction_data_msg.junction_node, junction_data_msg.destination_node = self.get_junction(graph, localisation)
        if self.previous_junction_node is not None and self.previous_junction_node != junction_data_msg.junction_node and self.command_is_being_processed:
            self.commands.pop(0)  # Remove the command once we've reached a new junction
            self.command_is_being_processed = False
        self.previous_junction_node = junction_data_msg.junction_node
        junction_data_msg.polar_coordinates_with_respect_to_car = self.GeneratePolarCoordinates(localisation, junction_data_msg.junction_node)
        

        self.pub.publish(junction_data_msg)
        self.get_logger().info(f"Published JunctionData")