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
from alns.select import RandomSelect
from alns.stop import MaxRuntime
import numpy.random as rnd
import copy
import matplotlib.pyplot as plt


def main():
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

def buildup_regular(grid_size: int=5, start: list=None, des: list=None, number_vehicle: int=10, distance: float=2.0, v: float=1.0):
    start_t = time.time()
    m = Model()
    # init
    edges = utils.generate_grid_edges(grid_size)
    edges_2dir = utils.to_2dir(edges)
    space_node = [i for i in range(grid_size*grid_size)]
    time_n2n = distance / v
    time_step = [i for i in range(int(2*grid_size*time_n2n))]
    time_space_node = [(i,j) for i in space_node for j in time_step]
    time_space_arc = [(i, j) for i in time_space_node for j in time_space_node if ((i != j) and (i[0],j[0]) in edges_2dir and (j[1] > i[1]))]

    # init var
    # x, y, z
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

    for (p,q) in edges_2dir:
    # select graph from pq
        for k in range(number_vehicle):
            ts_pq_arc = [(i,j) for (i,j) in time_space_arc if (i[0],j[0])==(p,q)]
            for t in time_step:
                # 如果不是j[1]-1就会出现01道路上，a车在时刻2从1出去，b车在时刻2从0进来，这种情况算堵车  
                m.addConstr(delta[p,q,t,k] == quicksum(x[i,j,k] for (i,j) in ts_pq_arc if (i[1]<=t and t<=j[1] - 1))) 
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

    # constrains for time
    tau = 1.5+0.01
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

    #constrains for degree
    for i in time_space_node:
        for k in range(number_vehicle):
            m.addConstr(quicksum(x[i,j,k] 
                for j in time_space_node if ((i[0],j[0]) in edges or (j[0],i[0]) in edges) and (j[1] > i[1])) 
            - quicksum(x[j,i,k] 
                for j in time_space_node if ((i[0],j[0]) in edges or (j[0],i[0]) in edges) and (j[1] < i[1])) == Y[i,k] - Z[i,k])    # 这地方可改可不改，意思都一样

    # constrains for Corner block based on degree
    r0 = 1
    for i in time_space_node:
        m.addConstr(beta[i] == quicksum(x[i,j,k] for j in time_space_node if (((i[0],j[0]) in edges_2dir) and (j[1] > i[1])) for k in range(number_vehicle)) 
                                - quicksum(Z[i,k] for k in range(number_vehicle))
        )
        m.addConstr(r[i]*M >= beta[i] - r0)
    m.addConstr(R == quicksum(r[i] for i in time_space_node))

    # constrains for Z
    for k in range(number_vehicle):
        m.addConstr(quicksum(Z[i,k] for i in time_space_node if i[0] != des[k]) == 0)
        m.addConstr(quicksum(Z[i,k] for i in time_space_node if i[0] == des[k]) == 1)

    m.addConstr(T == quicksum(x[i,j,k] * (j[1] - i[1]) for (i,j) in time_space_arc for k in range(number_vehicle)))

    alpha = 0.1
    gamma = 0.1
    #m.setObjective(T + alpha * B)
    m.setObjective(T + alpha * B +  gamma * R) # sense=gp.GRB.MAXIMIZE

    total_time = time.time() - start_t
    setCons_time = total_time - setV_time

    return m, total_time, setV_time, setCons_time

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
            if tmp_name > 0.5:
                i_tuple = ast.literal_eval(split_name[1])
                j_tuple = ast.literal_eval(split_name[2])
                k_ind = ast.literal_eval(split_name[3])
                x[k_ind].append((i_tuple, j_tuple))
            else:
                continue
    return x


    return None

if __name__ == '__main__':
    #main()
    grid_size = 5
    start = [(0,0),(2,1),(5,2),(11,3),(17,4),(23,5)]
    des = [4,22,24,4,0,11]
    m, total_time, setV_time, setCons_time = buildup_regular(grid_size=grid_size, start=start, des=des, number_vehicle=6)
    print(f'total_time:{total_time}, setV_time:{setV_time}, setCons_time:{setCons_time}')
    start_cal_time = time.time()
    m.optimize()
    cal_time = time.time() - start_cal_time
    print(f'cal_time:{cal_time}')