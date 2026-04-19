#!/usr/bin/env python3
import json


def load_voronoi_graph(path="voronoi_graph.json"):
    with open(path) as f:
        data = json.load(f)

    # nodes: {id: {x,y}}
    nodes = {int(k): v for k, v in data["nodes"].items()}

    # edges: [{"from": a, "to": b}]
    edges = data["edges"]

    # adjacency: generujemy pomocniczo
    adjacency = {i: [] for i in nodes.keys()}
    for e in edges:
        a = e["from"]
        b = e["to"]
        adjacency[a].append(b)
        adjacency[b].append(a)

    # intersections, dead_ends, inter_err — bierzemy dokładnie to, co dał saver
    intersections = {int(k): v for k, v in data.get("intersections", {}).items()}
    dead_ends = {int(k): v for k, v in data.get("dead_ends", {}).items()}
    inter_err = {int(k): v for k, v in data.get("inter_err", {}).items()}

    return {
        "nodes": nodes,
        "edges": edges,
        "adjacency": adjacency,
        "intersections": intersections,
        "dead_ends": dead_ends,
        "inter_err": inter_err
    }


if __name__ == "__main__":
    graph = load_voronoi_graph()
    print("Loaded graph:")
    print(f"- {len(graph['nodes'])} nodes")
    print(f"- {len(graph['edges'])} edges")
    print(f"- {len(graph['intersections'])} intersections (T/X/complex)")
    print(f"- {len(graph['dead_ends'])} dead ends")
    print(f"- {len(graph['inter_err'])} inter_err")
