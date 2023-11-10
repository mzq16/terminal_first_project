import os

def generate_grid_edges(grid_size):
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


def add2direction(space_arc:list):
    new_space_arc = []
    for tup in space_arc:
        new_space_arc.append((tup[1],tup[0]))
    return space_arc+new_space_arc