#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ros2py_voronoi.msg import Graph
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
import json


class VoronoiSaver(Node):
    def __init__(self):
        super().__init__("voronoi_saver")

        qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        self.subscription = self.create_subscription(
            Graph,
            "/voronoi_graph",
            self.callback,
            qos
        )

        self.graph_saved = False
        self.get_logger().info("VoronoiSaver running... waiting for /voronoi_graph")

    def callback(self, msg: Graph):
        if self.graph_saved:
            return

        self.get_logger().info(
            f"Received graph: {len(msg.nodes)} nodes, {len(msg.edges)} edges. Saving..."
        )

        # --- Węzły ---
        nodes = {}
        for idx, node in enumerate(msg.nodes):
            nodes[idx] = {"x": node.x, "y": node.y}

        # --- Krawędzie + sąsiedzi ---
        edges = []
        adjacency = {i: [] for i in nodes.keys()}

        for edge in msg.edges:
            a = edge.start_index
            b = edge.end_index

            # zapis krawędzi
            edges.append({"from": a, "to": b})

            # adjacency — tylko prawdziwi sąsiedzi
            adjacency[a].append(b)
            adjacency[b].append(a)

        # --- Klasyfikacja ---
        intersections = {}   # T, X, complex
        dead_ends = {}       # degree == 1
        inter_err = {}       # degree == 2 (błąd Voronoi)

        for node_id, neighbors in adjacency.items():
            degree = len(neighbors) - 1  # odejmujemy 1, bo w grafie Voronoi każdy węzeł jest połączony z samym sobą

            if degree == 1:
                dead_ends[node_id] = {
                    "x": nodes[node_id]["x"],
                    "y": nodes[node_id]["y"]
                }

            elif degree == 2:
                inter_err[node_id] = {
                    "x": nodes[node_id]["x"],
                    "y": nodes[node_id]["y"]
                }

            elif degree == 3:
                intersections[node_id] = {
                    "type": "T",
                    "x": nodes[node_id]["x"],
                    "y": nodes[node_id]["y"]
                }

            elif degree == 4:
                intersections[node_id] = {
                    "type": "X",
                    "x": nodes[node_id]["x"],
                    "y": nodes[node_id]["y"]
                }

            elif degree > 4:
                intersections[node_id] = {
                    "type": "complex",
                    "x": nodes[node_id]["x"],
                    "y": nodes[node_id]["y"]
                }

        # --- Zapis JSON ---
        graph = {
            "nodes": nodes,
            "edges": edges,
            "intersections": intersections,
            "dead_ends": dead_ends,
            "inter_err": inter_err
        }

        with open("voronoi_graph.json", "w") as f:
            json.dump(graph, f, indent=2)

        self.get_logger().info(
            f"Saved voronoi_graph.json with: "
            f"{len(intersections)} intersections (T/X/complex), "
            f"{len(dead_ends)} dead ends, "
            f"{len(inter_err)} inter_err."
        )

        self.graph_saved = True


def main(args=None):
    rclpy.init(args=args)
    node = VoronoiSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
