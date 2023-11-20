from gurobipy import *
from gurobipy import Model
from gurobipy import GRB
from gurobipy import quicksum
from gurobipy import min_
from gurobipy import max_
import gurobipy
import numpy as np
import os
import utils
import time
from collections import defaultdict
import ast
from alns import ALNS
from alns.accept import HillClimbing
from alns.select import RouletteWheel
from alns.select import RandomSelect
from alns.stop import MaxRuntime
import numpy.random as rnd
import copy
import matplotlib.pyplot as plt
from my_alns_class import ProblemState
from tqdm import tqdm
import pickle

# 上面都是GUROBI的代码
def test_GUROBI():
    # get random grid
    grid_size = 5
    start = [(0,0),(2,1),(5,2),(11,3),(17,4),(23,5)]
    des = [4,22,24,4,0,11]
    m, total_time, setV_time, setCons_time = buildup_regular(grid_size=grid_size, start=start, des=des, number_vehicle=6)
    print(f'total_time:{total_time}, setV_time:{setV_time}, setCons_time:{setCons_time}')
    start_cal_time = time.time()
    m.optimize()
    cal_time = time.time() - start_cal_time
    print(f'cal_time:{cal_time}')
    return m
    #x = get_vars(m=m)
    #return x

def buildup_regular(grid_size: int=5, start: list=None, des: list=None, 
                    number_vehicle: int=10, distance: float=2.0, v: float=1.0,
                    alpha = 1, gamma = 1):
    start_t = time.time()
    m = Model()
    # init
    edges = utils.generate_grid_edges(grid_size)
    edges_2dir = utils.to_2dir(edges)
    space_node = [i for i in range(grid_size*grid_size)]
    time_n2n = distance / v
    time_limit = 2 * time_n2n + 1
    # get latest task
    latest_time = max([i[1] for i in start])
    time_step = [i for i in range(int(2*grid_size*time_n2n+latest_time))]
    time_space_node = [(i,j) for i in space_node for j in time_step]
    # ts_arc could be exponential growth, in case of this we need add some limits
    # related to ex-growth: time limit, 超过limit意思就是车怎么着都过不去， 
    # not related to : t_i < t_j, i!=j, ij is connected in space
    time_space_arc = [(i, j) for i in time_space_node for j in time_space_node \
                      if ((i != j) and (i[0],j[0]) in edges_2dir and (j[1] > i[1]) and j[1] - i[1] < time_limit)]

    # init var
    # x, y, z
    print('start set Vars')
    x = {}
    for (i, j) in time_space_arc:
        for k in range(number_vehicle):
            x[i, j, k] = m.addVar(vtype=GRB.BINARY, name=f'x_{i}_{j}_{k}')
    Y = {}
    for i in time_space_node:
        for k in range(number_vehicle):
            if i == start[k]:
                Y[i,k] = 1
            else:
                Y[i,k] = 0
    Z = {}
    for i in time_space_node:
        for k in range(number_vehicle):
            Z[i,k] = m.addVar(vtype=GRB.BINARY, name=f'destination_{i}_for_{k}')
    
    # set block var
    delta = {}
    for (p,q) in edges_2dir:
        for t in time_step:
            for k in range(number_vehicle):
                delta[p,q,t,k] = m.addVar(vtype=GRB.BINARY, name=f'has_vehicle_on_{p}{q}_for_{k}_at_{t}')
    C = {}
    for (p,q) in edges_2dir:
        for t in time_step:
            C[p,q,t] = m.addVar(vtype=GRB.INTEGER, name=f'numberofvehicle_on_{p}{q}_at_{t}')

    b = {}
    for (p,q) in edges_2dir:
        for t in time_step:
            b[p,q,t] = m.addVar(vtype=GRB.BINARY, name=f'if_block_on_{p}{q}_at_{t}')

    bb = {}
    for t in time_step:
        bb[t] = m.addVar(vtype=GRB.INTEGER, name=f'numberofblock_at_{t}')

    B = m.addVar(vtype=GRB.INTEGER, name=f'block_number')

    T = m.addVar(vtype=GRB.INTEGER, name=f'total_time')

    # set other var
    # middle var for block counting
    f = {}
    ff = {}
    for (p,q) in edges_2dir:
        for t in time_step:
            f[p,q,t] = m.addVar(vtype=GRB.INTEGER, name=f'middle_on_{p}{q}_at_{t}')
            ff[p,q,t] = m.addVar(vtype=GRB.INTEGER, name=f'mmiddle_on_{p}{q}_at_{t}', lb=-GRB.INFINITY)

    # corner block
    r = {}
    beta = {}
    for i in time_space_node:
        r[i] = m.addVar(vtype=GRB.BINARY, name=f'block_at_{i}', lb=-GRB.INFINITY)
        beta[i] = m.addVar(vtype=GRB.INTEGER, name=f'number_of_vehicle_at_{i}', lb=-GRB.INFINITY)

    R = m.addVar(vtype=GRB.INTEGER, name=f'total_coner_block')

    # number of block between i & j
    l = {}
    for (i, j) in time_space_arc:
        l[i, j] = m.addVar(vtype=GRB.INTEGER, name=f'number_of_b_{i}&{j}')

    # middle var for multi
    z = {}
    for (i, j) in time_space_arc:
        for k in range(number_vehicle):
            z[i, j, k] = m.addVar(vtype=GRB.INTEGER, name=f'z_{i}_{j}_{k}',lb=-GRB.INFINITY)

    zz = {}
    for (i, j) in time_space_arc:
        for k in range(number_vehicle):
            zz[i, j, k] = m.addVar(vtype=GRB.INTEGER, name=f'z_{i}_{j}_{k}',lb=-GRB.INFINITY)

    m.update()

    
    setV_time = time.time() - start_t
    #print(total_time)

    '''
    -----------------------------------------
    |上面是设置变量，下面是约束               |
    |（1）堵车约束                           |
    |（2）时间约束                           |
    -----------------------------------------
    '''
    print('start set constraints')
    pb = tqdm(total=len(edges_2dir), initial=0)
    for (p,q) in edges_2dir:
    # select graph from pq
        for k in range(number_vehicle):
            ts_pq_arc = [(i,j) for (i,j) in time_space_arc if (i[0],j[0])==(p,q)]
            for t in time_step:
                # 如果不是j[1]-1就会出现01道路上，a车在时刻2从1出去，b车在时刻2从0进来，这种情况算堵车  
                m.addConstr(delta[p,q,t,k] == quicksum(x[i,j,k] for (i,j) in ts_pq_arc if (i[1]<=t and t<=j[1] - 1))) 
        pb.update(1)
    pb.close()

    
    for (p,q) in edges_2dir:
        for t in time_step:
            m.addConstr(C[p,q,t] == quicksum(delta[p,q,t,k] for k in range(number_vehicle)))
    
    b0 = 1 # block threshold
    M = 100
    '''
    for (p,q) in space_arc_2:
        for t in time_step:
            m.addConstr(b[p,q,t]*M >= C[p,q,t] - b0) 

    # 上面大M法有一些松弛，对于b来说永远可以取1，有好处坏处，好处变为速度控制，坏处则不可控
    # 下面用max min的方法对b夹逼一下，不给b松弛 需要引入新中间变量f
    
    这地方不减阈值堵车的时候就算不出来
    另外只能将变量常数传给max_ min_,因此gp.max_((C[p,q,t]+0), constant=0)) 写法是不行的,必须额外增加一个变量 
    https://support.gurobi.com/hc/en-us/community/posts/360078185112-gurobipy-Model-addGenConstrMin-Invalid-data-in-vars-array
    '''
    for (p,q) in edges_2dir:
        for t in time_step:
            m.addConstr(ff[p,q,t] == C[p,q,t]-b0)
            m.addConstr(f[p,q,t] == max_(ff[p,q,t], constant=0))            
            m.addConstr(b[p,q,t] == min_(f[p,q,t], constant=1))

    # constrain for sum
    for t in time_step:
        m.addConstr(quicksum(b[p,q,t] for (p,q) in edges_2dir) == bb[t])

    # constrain for sum 
    m.addConstr(B == quicksum(bb[t] for t in time_step))
    time_start = time.time()
    
    # constrains for time
    tau = distance / v
    print('start transfer speed to linear')
    pb = tqdm(total=len(x.keys()))
    for key in list(x.keys()):
        '''
        t_i = key[0][1]
        t_j = key[1][1]
        m.addConstr((t_j - t_i - 1) * x[key] <= 2)
        m.addConstr(t_j - t_i >= 2 * x[key])
        '''
        i = key[0]
        j = key[1]
        t_i = key[0][1]
        t_j = key[1][1]
        p = key[0][0]
        q = key[1][0]
        k = key[2]
        if t_j-t_i < 1:
            print('error!')
        m.addConstr(l[i,j] == quicksum(b[p,q,t] for t in range(t_i,t_j)))
        m.addConstr(z[i,j,k] <= 2*(t_j-t_i)-l[i,j])
        m.addConstr(z[i,j,k] >= 2*(t_j-t_i)-l[i,j] - 2*(t_j-t_i)*(1-x[key]))
        m.addConstr(z[i,j,k] >= (t_j-t_i)*x[key])
        m.addConstr(z[i,j,k] <= 2*(t_j-t_i)*x[key])
        m.addConstr(z[i,j,k] <= 2*tau*(t_j-t_i)/(t_j-t_i-1+1e-6))
        m.addConstr(t_j-t_i - l[i,j]/2 >= tau * x[key])
        pb.update(1)
    pb.close()
    t1 = time.time() - time_start
    
    #constrains for degree
    print('start add degree constrain')
    pb = tqdm(total=len(time_space_node))
    for i in time_space_node:
        for k in range(number_vehicle):
            m.addConstr(quicksum(x[i,j,k] for j in time_space_node if (i,j,k) in x.keys())
                # for j in time_space_node if ((i[0],j[0]) in edges or (j[0],i[0]) in edges) and (j[1] > i[1]))
            - quicksum(x[j,i,k] for j in time_space_node if (j,i,k) in x.keys()) == Y[i,k] - Z[i,k])    # 这地方可改可不改，意思都一样
                #for j in time_space_node if ((i[0],j[0]) in edges or (j[0],i[0]) in edges) and (j[1] < i[1])) == Y[i,k] - Z[i,k])    # 这地方可改可不改，意思都一样
        pb.update(1)
    pb.close()      
    t2 = time.time() - time_start - t1
    
    # constrains for Corner block based on degree
    r0 = 1
    print('start calculate node degree')
    pb = tqdm(total=len(time_space_node))
    for i in time_space_node:
        #m.addConstr(beta[i] == quicksum(x[i,j,k] for j in time_space_node if (((i[0],j[0]) in edges_2dir) and (j[1] > i[1])) for k in range(number_vehicle)) 
        m.addConstr(beta[i] == quicksum(x[i,j,k] for j in time_space_node for k in range(number_vehicle) if (i,j,k) in x.keys()) 
                                - quicksum(Z[i,k] for k in range(number_vehicle))
        )
        m.addConstr(r[i]*M >= beta[i] - r0)
        pb.update(1)
    pb.close()
    m.addConstr(R == quicksum(r[i] for i in time_space_node))
    t3 = time.time() - time_start - t1 - t2
    
    # constrains for Z
    for k in range(number_vehicle):
        m.addConstr(quicksum(Z[i,k] for i in time_space_node if i[0] != des[k]) == 0)
        m.addConstr(quicksum(Z[i,k] for i in time_space_node if i[0] == des[k]) == 1)

    m.addConstr(T == quicksum(x[i,j,k] * (j[1] - i[1]) for (i,j) in time_space_arc for k in range(number_vehicle)))

    #m.setObjective(T + alpha * B)
    m.setObjective(T + alpha * B +  gamma * R) # sense=gp.GRB.MAXIMIZE
    
    ttt = [t1,t2,t3]
    total_time = time.time() - start_t
    setCons_time = total_time - setV_time

    return m, total_time, setV_time, setCons_time, ttt

def get_vars(m:gurobipy.Model):
    '''
    could find the name formation in the buildup function
    name=f'x_{i}_{j}_{k}', e.g. 'x_(0,1)_(0,2)_1'
    '''
    x = defaultdict(list)  # x['1'] = [((0,1),(1,2)),((1,2),(2,3)), ...] 
    for v in m.getVars():
        tmp_name = v.varName
        tmp_value = v.x
        split_name = tmp_name.split('_')
        if len(split_name) == 0:
            continue
        elif split_name[0] == 'x':
            if tmp_value > 0.5:
                i_tuple = ast.literal_eval(split_name[1])
                j_tuple = ast.literal_eval(split_name[2])
                k_ind = ast.literal_eval(split_name[3])
                x[k_ind].append((i_tuple, j_tuple))
            else:
                continue
    return x

# 下面都是启发式算法的代码
def get_init_route():
    pass

def buildup_alns_regular(start: list=None, des: list=None, v_spd: float=2.0, 
                         distance: list=[], space_arc: list=[]) -> ALNS:
    # init problem state
    v_r = None
    state = ProblemState(vehicle_routes = v_r, destinations = des, start_location = start, 
                         vehicle_speed = v_spd, space_distance = distance, space_arc = space_arc)
    return state

def test_heuristic():
    start = [(0,0),(2,1),(5,2),(11,3),(17,4),(23,5)]
    des = [4,22,24,4,0,11]
    grid_size = 5

    # init state
    t0 = time.time()
    problem_state = init_problem_state(start=start, des=des, grid_size=grid_size, v_spd=1.0, dist=2.0)
    init_time = time.time() - t0
    print(f'inital time: {init_time}')
    rnd_state = np.random.RandomState(42)

    # Create ALNS and add one or more destroy and repair operators
    alns = ALNS(rnd.RandomState(seed=42))
    alns.add_destroy_operator(random_destroy)
    alns.add_repair_operator(random_repair)

    # Configure ALNS
    select = RandomSelect(num_destroy=1, num_repair=1)  # see alns.select for others
    accept = HillClimbing()  # see alns.accept for others
    iter_time = 10
    stop = MaxRuntime(iter_time)  # 60 seconds; see alns.stop for others

    # Run the ALNS algorithm
    print(f'start calculating for {iter_time} seconds ')
    result = alns.iterate(problem_state, select, accept, stop)

    # Retrieve the final solution
    best = result.best_state
    print(f"Best heuristic solution objective is {best.objective()}.")

    return result
    
def init_problem_state(start:list, des:list, grid_size:int, v_spd:float, dist:float) -> ProblemState:
    OD_pairs = utils.get_OD_pair(start=start, des=des)
    space_routes = defaultdict(list)
    # get graph
    edges = utils.generate_grid_edges(grid_size=grid_size)
    edges_2dir = utils.to_2dir(edges)
    # get TS routes
    for i in range(len(OD_pairs)):
        tmp_path = utils.get_space_path(start=OD_pairs[i][0], des=OD_pairs[i][1], bi_space_arc=edges_2dir)
        tmp_route = utils.path_to_route(tmp_path)
        space_routes[i] = tmp_route
    
    # get graph distance
    graph_distance = utils.get_graph_dist(edges_2dir=edges_2dir, dist=dist)

    # get init state 
    problem_state = ProblemState(space_routes = space_routes, destinations = des, start_location = start, 
                         vehicle_speed = v_spd, space_distance = graph_distance, space_arc = edges_2dir)
    return problem_state

def setup_alns(seed=4256, iter_time=10):
    # Create ALNS and add one or more destroy and repair operators
    alns = ALNS(rnd.RandomState(seed=seed))
    alns.add_destroy_operator(greedy_destroy, 'greedy')
    alns.add_destroy_operator(random_destroy, 'random')
    

    alns.add_repair_operator(random_repair)
    #alns.add_repair_operator(normal_repair)

    # Configure ALNS
    select = RandomSelect(num_destroy=2, num_repair=1)  # see alns.select for others
    # select = RouletteWheel([3, 2, 1, 0.5], 0.8, 2, 2)
    accept = HillClimbing()  # see alns.accept for others
    iter_time = iter_time
    stop = MaxRuntime(iter_time)  # 60 seconds; see alns.stop for others
    return alns, select, accept, stop

# destroy
def destroy(current: ProblemState, rnd_state: rnd.RandomState) -> ProblemState:
    # TODO implement how to destroy the current state, and return the destroyed
    #  state. Make sure to (deep)copy the current state before modifying!
    pass

def random_destroy(current: ProblemState, rnd_state: rnd.RandomState) -> ProblemState:
    destroy_number = 1
    next_state = copy.deepcopy(current)
    destroy_ids = rnd_state.choice(np.arange(0, len(current.vehicle_routes)), destroy_number, replace=False)
    next_state.update_from_destroy(destroy_ids=list(destroy_ids))
    return next_state

def greedy_destroy(current: ProblemState, rnd_state: rnd.RandomState) -> ProblemState:
    next_state = copy.deepcopy(current)
    choice_list = current.get_greedy_destroy_ids()
    choice_index = rnd_state.choice(len(choice_list))
    destroy_id = choice_list[choice_index][0]
    next_state.update_from_destroy(destroy_ids=[destroy_id])
    return next_state
    
#repair
def repair(destroyed: ProblemState, rnd_state: rnd.RandomState) -> ProblemState:
    # TODO implement how to repair a destroyed state, and return it
    pass

def random_repair(destroyed: ProblemState, rnd_state: rnd.RandomState) -> ProblemState:
    ttt = destroyed.update_from_random_repair(rnd_state=rnd_state)
    return destroyed

def normal_repair(destroyed: ProblemState, rnd_state: rnd.RandomState) -> ProblemState:
    destroy_nos = destroyed.get_destroy_nos()
    ttt = destroyed.update_from_normal_repair(destroy_nos)
    return destroyed

def greedy_repair(destroyed: ProblemState, rnd_state: rnd.RandomState) -> ProblemState:
    pass

def main():
    num_exp = 50
    experiments_data = []
    for exp_id in range(1, num_exp + 1):
        print('-------------------------------------------------------')
        print(f'start {exp_id} experiment')
        n_vehicle = 50
        grid_size = 8
        dist = 2.0
        v_spd = 1.0
        
        # generate random OD
        ODs = []
        for _ in range(n_vehicle):
            options = np.arange(start=0, stop=grid_size ** 2 - 1)
            tmp_od = np.random.choice(options, 2,replace=False)
            ODs.append(tuple(tmp_od))
        start_loc, des = zip(*ODs)
        times = tuple(np.random.randint(0,10,(n_vehicle,)))
        start = list(zip(start_loc,times))


        # gurobi
        m, total_time, setV_time, setCons_time, ttt\
            = buildup_regular(grid_size=grid_size, start=start, des=des, number_vehicle=n_vehicle, 
                            distance=dist, v=v_spd)
        m.setParam('OutputFlag', 0)
        print(f'total_time:{total_time}, setV_time:{setV_time}, setCons_time:{setCons_time}')
        print(ttt)
        print('start optimize')
        start_gurobi_time = time.time()
        m.optimize()
        cal_time = time.time() - start_gurobi_time
        print(f'cal_time:{cal_time}')
        result_x = get_vars(m=m)
        gurobi_time = {
            'opt_time':cal_time,
            'total_time':total_time,
            'setV_time':setV_time,
            'setCons_time':setCons_time,
            'setCons_details':ttt,
        }

        experiment_data = {
            'experiment_id':exp_id,
            'result_route_gurobi':result_x,
            'start_node':start,
            'destination':des,
            'vehicle_speed':v_spd,
            'distance':dist,
            'grid_size':grid_size,
            'times':gurobi_time,
        }

        with open(f'experiment_{exp_id}.pkl', 'wb') as file:
            pickle.dump(experiment_data, file)
        experiments_data.append(experiment_data)
    
    # finish loop
    with open(f'all_experiments.pkl', 'wb') as file:
        pickle.dump(experiments_data, file)
        # alns 
        '''
        t0 = time.time()
        problem_state = init_problem_state(start=start, des=des, grid_size=grid_size, v_spd=v_spd, dist=dist)
        init_time = time.time() - t0
        print(f'inital time: {init_time}')

        alns, select, accept, stop = setup_alns()
        result = alns.iterate(problem_state, select, accept, stop)

        # Retrieve the final solution
        best = result.best_state
        print(f"Best heuristic solution objective is {best.objective()}.")
        gurobi_best = m.objVal
        print(f'gurobi:{gurobi_best}, alns:{best.objective()}, {best.objective()/gurobi_best}')
        '''


if __name__ == '__main__':
    #m = test_GUROBI()
    #ps = test_heuristic()
    m = main()