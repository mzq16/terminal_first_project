import os
import heapq
from collections import deque
from collections import defaultdict

def generate_grid_edges(grid_size) -> list:
    if grid_size < 1:
        return []

    grid_edges = []
    node_count = grid_size * grid_size

    for i in range(grid_size):
        for j in range(grid_size):
            node = i * grid_size + j

            # Right neighbor
            if j < grid_size - 1:
                right_neighbor = i * grid_size + (j + 1)
                grid_edges.append((node, right_neighbor))

            # Bottom neighbor
            if i < grid_size - 1:
                bottom_neighbor = (i + 1) * grid_size + j
                grid_edges.append((node, bottom_neighbor))

    return grid_edges

def to_2dir(space_arc: list) -> list:
    new_space_arc = []
    for tup in space_arc:
        new_space_arc.append((tup[1],tup[0]))
    return space_arc + new_space_arc

def get_neighbour_point(current_point, bi_space_arc) -> list:
    neighbour_point = []
    for arc in bi_space_arc:
        if current_point == arc[0]:
            neighbour_point.append(int(arc[1]))
    return neighbour_point

def get_space_path(start: int=0, des: int=10, bi_space_arc: list=[]) -> list:
    q = deque([(start, [start])])    # (node, path)
    visited = set()
    visited.add(start)
    while q:
        curr_node, curr_path = q.popleft()
        if curr_node == des:
            return curr_path
        curr_neighbours =  get_neighbour_point(current_point=curr_node, bi_space_arc=bi_space_arc)
        for nei in curr_neighbours:
            if nei not in visited:
                visited.add(nei)
                q.append((nei, curr_path + [nei]))
    return []

def get_timespace_path_first(space_path: list, distance: float, spd: float):
    time = 1

def get_OD_pair(start: list, des: list):
    start_loc = [i[0] for i in start]    
    return list(zip(start_loc, des))

def get_graph_dist(edges_2dir: list, dist: float) -> dict:
    distance = {}
    for i in range(len(edges_2dir)):
        distance[edges_2dir[i]] = dist
    return distance
        
def path_to_route(single_path: list):
    tmp_route = []
    for i in range(len(single_path) - 1):
        tmp_route.append((single_path[i], single_path[i+1]))
    return tmp_route

def dijkstra(graph, start):
    """
    Dijkstra's algorithm to find the shortest paths from a starting node to all other nodes in a weighted graph.

    :param graph: Dictionary representing the graph with nodes as keys and a list of (neighbor, weight) tuples as values.
    :param start: Starting node.
    :return: Dictionary of shortest distances from the start node to all other nodes.

    PS: dijs是适用于有向无环图，对于无向图，一根无向的边就可以被视作是一个环，
    所以对于本路网是不适用的
    """

    # Priority queue to store (distance, node) pairs
    priority_queue = [(0, start)]
    # Dictionary to store the shortest distances
    distances = {node: float('infinity') for node in graph}
    distances[start] = 0

    while priority_queue:
        current_distance, current_node = heapq.heappop(priority_queue)

        # Check if the current distance is already greater than the known shortest distance
        if current_distance > distances[current_node]:
            continue

        for neighbor, weight in graph[current_node]:
            distance = current_distance + weight

            # If a shorter path is found, update the distance and push it to the priority queue
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                heapq.heappush(priority_queue, (distance, neighbor))

    return distances


