#!/usr/bin/env python3

from math import atan2
import math

import rclpy
from rclpy.node import Node

import pyvoronoi
from ros2py_voronoi.msg import Graph, GraphNode, GraphEdge, JunctionData, PolarCoordinates
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy


class PolarCoordinatesCOnverter(Node):
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
        self.pub = self.create_publisher(JunctionData, '/junction_data', map_qos)

        self.get_logger().info("Junction Semantics Node Started. Waiting for /voronoi_graph and /localisation...")
        self.latest_localisation = None
        self.latest_graph = None
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
    def get_junction(Graph,car_coordinates):
        car_orientation = car_coordinates.orientation
        car_x = car_coordinates.x
        car_y = car_coordinates.y
        closest_edge = None
        closest_edge_distance = float('inf')
        for edge in Graph.edges:
            start_node = Graph.nodes[edge.start_index]
            end_node = Graph.nodes[edge.end_index]
            # Check if the car is close to the edge (within a certain threshold)
            distance_to_edge = self.distance_point_to_line(start_node, end_node,car_x, car_y)
            if distance_to_edge < closest_edge_distance:
                closest_edge_distance = distance_to_edge
                closest_edge = edge
        edge1_angular_distance = abs(car_orientation - atan2(start_node.y - car_y, start_node.x - car_x))
        edge2_angular_distance = abs(car_orientation - atan2(end_node.y - car_y, end_node.x - car_x))
        if edge1_angular_distance < edge2_angular_distance:
            return start_node
        else:            
            return end_node
    def GeneratePolarCoordinates(car_coordinates,junction_coordinates)
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
        if graph is None or localisation is None:
            return  # Wait until both messages are received
                # Publish the junction semantics data
        junction_data_msg = JunctionData()
        junction_data_msg.header.stamp = self.get_clock().now().to_msg()
        junction_data_msg.polar_coordinates_with_respect_to_car = self.get_junction(graph, localisation)
        junction_data_msg.junction_node =  = self.GeneratePolarCoordinates(localisation, junction_coordinates)

        self.pub.publish(junction_data_msg)
        self.get_logger().info(f"Published JunctionData")