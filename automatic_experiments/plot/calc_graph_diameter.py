from collections import deque
import math
from plot import *
import pathlib

def bfs_farthest_distance(graph, start):
    visited = set()
    queue = deque([(start, 0)])  # (current_node, current_distance)
    max_distance = 0

    while queue:
        node, distance = queue.popleft()

        if node not in visited:
            visited.add(node)
            max_distance = max(max_distance, distance)

            for neighbor in graph[node]:
                if neighbor not in visited:
                    queue.append((neighbor, distance + 1))

    return max_distance

def calculate_diameter(graph):
    diameter = 0

    for node in graph:
        farthest_distance = bfs_farthest_distance(graph, node)
        diameter = max(diameter, farthest_distance)

    return diameter


def calculate_distance(point1, point2):
    return math.sqrt(sum((p1 - p2) ** 2 for p1, p2 in zip(point1, point2)))

def construct_graph(points, range_distance):
    graph = {i: [] for i in range(len(points))}

    for i in range(len(points)):
        for j in range(i + 1, len(points)):
            if calculate_distance(points[i], points[j]) < range_distance:
                graph[i].append(j)
                graph[j].append(i)

    return graph

if __name__ == "__main__":
    # pos = [[-0.025437,0.548458],[0.425501,0.737967],[0.069772,0.736306],[0.185392,0.418015],[0.275733,0.144281],[-0.642936,-0.522379],[-0.591091,-0.085356],[0.707292,-0.746031],[-0.651168,0.006991],[0.522154,-0.742039],[0.039299,-0.034494],[0.413981,-0.576272],[0.066500,0.083064],[0.338490,-0.281079],[-0.405552,-0.200177],[-0.214112,-0.057548],[-0.246196,-0.184708],[-0.492586,-0.655497],[-0.263204,0.100512],[-0.103505,0.498102]]
    directory_path = "/home/lior/workspace/robust-matching/automatic_experiments/results/final/connected/range_2_robots_80/algo_matching_crash_0_1/faulty0"
    files = [f for f in pathlib.Path(directory_path).iterdir() if f.is_file() and ".log" in f.name]
    diameters = []
    for i, file in enumerate(files):
        a = open_file(file)
        pos = a["init_positions"]
        graph = construct_graph(pos, range_distance=0.5)
        diameter = calculate_diameter(graph)
        diameters.append(diameter)
    d = get_std_mean(diameters)
    print(d)
        