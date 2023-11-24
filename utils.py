import os
import heapq
from collections import deque
from collections import defaultdict
from my_alns_class import ProblemState
import numpy as np
import numpy.random as rnd

# graph function
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

def generate_poiny_xy(grid_size):
    point_xy = {}
    for i in range(grid_size):
        for j in range(grid_size):
            point_xy[i*grid_size+j] = np.array([j,i])
    return point_xy

# get function
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

# transfer function
def path_to_route(single_path: list):
    tmp_route = []
    for i in range(len(single_path) - 1):
        tmp_route.append((single_path[i], single_path[i+1]))
    return tmp_route

# test function
def compare_2methods_result(gurobi_data):
    des = gurobi_data['destination']
    start = gurobi_data['start_node']
    v_spd = gurobi_data['vehicle_speed']
    edges = generate_grid_edges(grid_size=gurobi_data['grid_size'])
    edges_2dir = to_2dir(edges)
    ts_routes = sort_ts_routes(gurobi_data['result_route_gurobi'])
    graph_distance = get_graph_dist(edges_2dir=edges_2dir,dist=gurobi_data['distance'])
    problem_state = ProblemState(time_space_routes = ts_routes, destinations = des, start_location = start, 
                            vehicle_speed = v_spd, space_distance = graph_distance, space_arc = edges_2dir)
    return True if gurobi_data['gurobi_obj'] == problem_state.objective() else False

def sort_ts_routes(multi_ts_route: dict):
    ts_routes = {}
    for k in multi_ts_route.keys():
        t_list = multi_ts_route[k]
        tmp_list = sorted(t_list, key=lambda x:x[0][1])
        ts_routes[k] = tmp_list
    return ts_routes

def cal_alns_obj_from_gurobi(gurobi_data):
    des = gurobi_data['destination']
    start = gurobi_data['start_node']
    v_spd = gurobi_data['vehicle_speed']
    edges = generate_grid_edges(grid_size=gurobi_data['grid_size'])
    edges_2dir = to_2dir(edges)
    ts_routes = sort_ts_routes(gurobi_data['result_route_gurobi'])
    graph_distance = get_graph_dist(edges_2dir=edges_2dir,dist=gurobi_data['distance'])
    point_xy = {}
    grid_size = gurobi_data['grid_size']
    for i in range(grid_size):
        for j in range(grid_size):
            point_xy[i*grid_size+j] = np.array([j,i])
    problem_state = ProblemState(time_space_routes = ts_routes, destinations = des, start_location = start, 
                            vehicle_speed = v_spd, space_distance = graph_distance, space_arc = edges, point_xy=point_xy)
    return problem_state.objective()

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
        # 思路就是队列新来一个点，如果比我们有的大，那就不管，continue
        # 然后如果比我们的小，就管一下：
        # 比我们现有的小，那么我们先更新一下这个点的neighbour
        # 没更新的就不变，更新了的就加入到队列中
        # 那么队列的意思就是等待更新的，当队列清空了，意味着没有需要更新的
        if current_distance > distances[current_node]:          # 这地方有点误解，distances里面才是我们先有的，curr那个其实是队列新来的
            continue

        for neighbor, weight in graph[current_node]:
            distance = current_distance + weight

            # If a shorter path is found, update the distance and push it to the priority queue
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                heapq.heappush(priority_queue, (distance, neighbor))

    return distances

def get_simple_dist(point_a:np.ndarray, point_b:np.ndarray):
    return sum(abs(point_a - point_b))

# 这两个norm的函数跑1e5次，时间：0.5090627670288086 0.37665748596191406，可以看到多次交换确实浪费时间
def normalize_weight_slow(dist:list, ratio:list = [4,3,2,1]):
    ratio = np.array(ratio)[:len(dist)]
    ratio_norm = ratio / sum(ratio)
    indexed_dist = list(enumerate(dist))                                        # [(0,a),(1,b),(2,c)]
    sorted_dist = sorted(indexed_dist, key= lambda x: x[1])                     # [(1,b),(2,c),(0,1)] b>c>a
    element_ranks = [rank for rank, _ in sorted(list(enumerate(sorted_dist)), key=lambda x: x[1][0])]  # [(0,(1,b)),(1,(2,c)),(2,(0,a))] ------> [(2,(0,a)),(0,(1,b)),(1,(2,c))]
    return [ratio_norm[element_ranks[i]]for i in range(len(ratio_norm))]

def normalize_weight(dist:list, ratio:list = [4,3,2,1]):
    output = np.zeros(len(dist))
    ratio = np.array(ratio)[:len(dist)]
    ratio_norm = ratio / sum(ratio)
    indexed_dist = list(enumerate(dist))
    sorted_dist = sorted(indexed_dist, key= lambda x: x[1])
    p = 0
    for index, _ in sorted_dist:
        output[index] = ratio_norm[p]
        p += 1
    return output

def get_space_path_onesteplook(edges_2dir: list, graph_dist_2dir:dict, point_xy, v_spd, road_block:defaultdict, 
                               point_block:defaultdict, start:tuple, des:int, rnd_state:rnd.RandomState, a=1.5,b=0.3):
    timestep = start[1]
    arrived = False
    space_path = [start[0]]
    current_point = start[0]
    prev_point = start[0]
    while not arrived:
        neighbour_points = get_neighbour_point(current_point=current_point, bi_space_arc=edges_2dir)
        # check arrived
        if des in neighbour_points:
            arrived = True
            space_path.append(des)
            break
        # check neighbour if only got one
        if len(neighbour_points) == 1:
            space_path.append(neighbour_points[0])
            tmp_road = (current_point, neighbour_points[0])
            normal_time = graph_dist_2dir[tmp_road] / v_spd
            timestep += int(np.ceil(normal_time))
            continue

        # not arrived, check every option weight
        # if restrict the direction
        if prev_point in neighbour_points:
            neighbour_points.remove(prev_point)

        # start infer
        block_penalty = [0 for _ in range(len(neighbour_points))]
        point_penalty = [0 for _ in range(len(neighbour_points))]
        route_penalty = []
        for i in range(len(neighbour_points)):
            tmp_road = (current_point, neighbour_points[i])
            normal_time = int(np.ceil(graph_dist_2dir[tmp_road] / v_spd))
            for tt in range(normal_time):
                block_penalty[i] += road_block[tmp_road][timestep + tt]
            point_penalty[i] += point_block[(neighbour_points[i], timestep + normal_time)]
            route_penalty.append(get_simple_dist(point_xy[neighbour_points[i]], des))
        route_weight = normalize_weight(route_penalty)
        block_weight = normalize_weight(block_penalty)
        point_weight = normalize_weight(point_penalty)
        total_weight = route_weight + a * block_weight + b * point_weight
        weight_norm = total_weight / sum(total_weight)
        select_point = int(rnd_state.choice(neighbour_points, size=1, p=weight_norm))
        space_path.append(select_point)
        prev_point = current_point
        current_point = select_point
        timestep += normal_time
    return space_path

def get_TSspace_path_greedy(grid_size:int, edges_2dir:list, graph_dist_2dir:dict, v_spd:float, road_traffic:defaultdict, 
                          point_traffic:defaultdict, start:tuple, des:int, b0:int, p0:int):
    start_node = start[0]
    start_time = start[1]
    path_nodes = {}                      # {1:[(space, time)]}
    penalty_nodes = {}                 # {1:0}
    for i in range(grid_size**2):
        path_nodes[i] = []
        penalty_nodes[i] = float(np.inf)
    path_nodes[start_node].append(start)
    penalty_nodes[start_node] = 0
    current_point = start_node
    prev_point = start_node
    visited = set([start_node])
    wait4update = set([i for i in range(grid_size**2)])
    wait4update.remove(start_node)

    '''
    思路就是管理两个set，一个是visited，一个是没有，然后while none-visited
    做一个生成树，每次拉一个距离树最小距离的节点，然后更新邻居，只更新邻居，是否更新visited里面的就不清楚了
    原dijkstra是针对有向无环，所以没有这个问题

    '''
    while wait4update:
        neighbour_points = get_neighbour_point(current_point=current_point, bi_space_arc=edges_2dir)
        if prev_point in neighbour_points:
            neighbour_points.remove(prev_point)
        tmp_t = path_nodes[current_point][-1][1]
        # update all new dist
        # 是否会更新更新了点的邻居
        # 如果也更新的话，就会一直更新下去直到最优，时间开销不一定很容易搞定
        # 但其实也不一定，因为算上时间的penalty，基本上稍微绕绕圈子开penalty就不会小，所以不会更新
        for neighbour_p in neighbour_points:
            tmp_road = (current_point, neighbour_p)
            # check if block 
            tmp_block_t = int(np.ceil(2 * graph_dist_2dir[tmp_road] / v_spd))
            block_list = [road_traffic[tmp_road][tmp_t + i] for i in range(tmp_block_t)]
            pass_time, block_situation = cal_pass_time(block_list, graph_dist_2dir[tmp_road], v_spd, b0)
            block_n = sum(block_situation)
            is_point_block = point_traffic[(neighbour_p, tmp_t + pass_time)] + 1 > p0
            new_penalty = penalty_nodes[current_point] + is_point_block * (point_traffic[(neighbour_p, tmp_t + pass_time)] + 1) + pass_time + block_n
            # compare and update
            if penalty_nodes[neighbour_p] > new_penalty:
                penalty_nodes[neighbour_p] = new_penalty
                path_nodes[neighbour_p] = path_nodes[current_point] + [(neighbour_p, tmp_t + pass_time)]
                # 如果更新，就说明这个点的相邻点也需要更新，因此重新放入更新池中，如果时间太长，这一段可以注释
                if neighbour_p in visited and neighbour_p != start_node:
                    visited.remove(neighbour_p)
                    wait4update.add(neighbour_p)
        # 传入lambda中的x是dict的key，如果写成d.items()，这样传入x的就是（key,value)
        # e.g. min([(k,v) for k,v in penalty_nodes.items()]，key = lambda x: x[1])
        #   min_pen_node = min(penalty_nodes, key = lambda x: penalty_nodes[x]) 这两个是等价
        # 推测dict 传过去是key， list 传过去是元素
        # 我也不知道这是为啥，不过感觉写出来更灵活一些，下面就是这个例子
        min_penalty_node, penalty = min([(k,v) for k,v in penalty_nodes.items() if k in wait4update],key = lambda x: x[1])
        # 我们从wait for update中的点挑选一个距离最近的点，来更新周围的点，
        visited.add(min_penalty_node)
        wait4update.remove(min_penalty_node)
        current_point = min_penalty_node
    # all visited
    return path_nodes[des]
    
def cal_pass_time(block_:list, road_dist:float, speed:float, b0:float):
    tmp_d = 0
    b_situation = []
    for i in range(len(block_)):
        if tmp_d >= road_dist:
            break
        if block_[i] + 1 > b0:
            tmp_d += speed / 2
            b_situation.append(block_[i] + 1)
        else:
            tmp_d += speed
            b_situation.append(0)
    return i, np.array(b_situation)
         