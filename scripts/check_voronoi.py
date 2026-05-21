import matplotlib.pyplot as plt
from voronoi_loader import load_voronoi_graph

g = load_voronoi_graph()

nodes = g["nodes"]
edges = g["edges"]
intersections = g["intersections"]
dead_ends = g["dead_ends"]
inter_err = g["inter_err"]

plt.figure(figsize=(10,10))

# krawędzie
for e in edges:
    a = nodes[e["from"]]
    b = nodes[e["to"]]
    plt.plot([a["x"], b["x"]], [a["y"], b["y"]], 'g-', linewidth=1)

# węzły
for node_id, data in nodes.items():
    plt.scatter(data["x"], data["y"], c='red', s=10)

# skrzyżowania z typami
for node_id, data in intersections.items():
    plt.text(data["x"], data["y"], data["type"], fontsize=8)

plt.title("Voronoi Graph with Intersection Types")
plt.xlabel("X")
plt.ylabel("Y")
plt.axis("equal")
plt.show()
