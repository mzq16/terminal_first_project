from alns import ALNS
from alns.accept import HillClimbing
from alns.select import RandomSelect
from alns.stop import MaxRuntime

import numpy as np
import numpy.random as rnd
import copy
from collections import defaultdict
import matplotlib.pyplot as plt
import utils

class ProblemState:
    # TODO add attributes that encode a solution to the problem instance
    def __init__(self, 
                 vehicle_routes: defaultdict(list) = None, 
                 destinations: list = None, 
                 start_location: list = None, 
                 vehicle_speed: float = 1.0,
                 space_distance: float = 2.0,
                 space_arc: list = None,
                 random_size = None,
                 b0: int = 1, 
                 p0: int = 1,
                 init_flag = True) -> None:
        self.vehicle_routes = vehicle_routes
        self.destinations = destinations
        self.start_location = start_location
        # dict 内嵌 dict, e.g. road_traffic['a']['b'] = 1, start from 0
        self.road_traffic = defaultdict(lambda: defaultdict(int))   # r_t[road][t] --------> r_t[p1,p2][t] = v_number
        self.point_traffic = defaultdict(int)  # p_t[point][t] --------> p_t[p1][t] = v_number
        self.vehicle_time = defaultdict(int)                        # v_t[v] = total_time
        self.road_block = defaultdict(lambda: defaultdict(int))     # r_b[road][t] = 1 if block else 0
        self.point_block = defaultdict(int)    # p_b[point][t] = 1 if block else 0
        self.vehicle_speed = vehicle_speed
        self.space_distance = space_distance
        self.space_arc = space_arc
        self.b0 = b0
        self.p0 = p0
        if init_flag:
            self.init_update_variable_value()
        self.T = 0
        self.B = 0
        self.P = 0
        self.destroy_nos = np.array([]) # ndarray
        self.bi_space_arc =  self.get_bidirection_arc()

    def init_update_variable_value(self) -> None:
        '''
        3 parts: T, B & R
        T: total time for tasks 
        B: block number during the finish time
        R: corner block number
        '''
        
        for vehicle, arcs in self.vehicle_routes.items():
            previous_TS_point = None
            for arc in arcs:
                # if previous_TS_point == None:
                previous_TS_point = arc[0]  # (space,time)
                next_TS_point = arc[1]      # (space,time)

                # (1) cal t
                self.vehicle_time[vehicle] += next_TS_point[1] - previous_TS_point[1]

                # (2) cal b
                tmp_road = (previous_TS_point[0], next_TS_point[0])
                for t in range(previous_TS_point[1], next_TS_point[1]):
                    self.road_traffic[tmp_road][t] += 1

                # (3) cal p
                # self.point_traffic[previous_TS_point[0]][previous_TS_point[1]] += 1
                self.point_traffic[next_TS_point] += 1
        
        # (4) cal road_block         
        for road, traffic_time in self.road_traffic.items():
            for t, vehicle_number in traffic_time.items():
                if vehicle_number > self.b0:
                    self.road_block[road][t] =1
        
        # (5) cal point_block
        for TS_point, vehicle_number in self.point_traffic.items():
            if vehicle_number > self.p0:
                self.point_block[TS_point] = 1

    def objective(self) -> float:
        # TODO implement the objective function
        T = 0
        B = 0
        P = 0
        # (1) T
        for _, time in self.vehicle_time.items():
            T += time
        
        # (2) B
        for road, traffic_time in self.road_block.items():
            for t, block_flag in traffic_time.items():
                B += block_flag
                    
        # (3) P
        for TS_point, block_flag in self.point_block.items():
            P += block_flag
                     
        # fianl
        a1=a2=1.0
        self.T = T
        self.B = B
        self.P = P
        return T + a1 * B + a2 * P

    def deep_copy(self):
        return copy.deepcopy(self.vehicle_routes), copy.deepcopy(self.vehicle_time), copy.deepcopy(self.road_traffic), \
            copy.deepcopy(self.point_traffic), copy.deepcopy(self.road_block), copy.deepcopy(self.point_block)

    def update_from_destroy(self, destroy_nos: list):
        self.destroy_nos = destroy_nos
        for destroy_no in destroy_nos:
            assert destroy_no in self.vehicle_routes, f'delete number {destroy_no} does not in the vehicle routes'
            for arc in self.vehicle_routes[destroy_no]:
                # (3)r_t
                tmp_road = (arc[0][0], arc[1][0])
                for t in range(arc[0][1], arc[1][1]):
                    self.road_traffic[tmp_road][t] -= 1
                # (4)p_t
                self.point_traffic[arc[1]] -= 1
        # (5)r_b, (6)p_b 
        for destroy_no in self.destroy_nos:
            for arc in self.vehicle_routes[destroy_no]:
                tmp_road = (arc[0][0],arc[1][0])
                for t in range(arc[0][1], arc[1][1]):
                    if self.road_traffic[tmp_road][t] <= self.b0:
                        self.road_block[tmp_road][t] = 0
                if self.point_traffic[arc[1]] <= self.p0:
                    self.point_block[arc[1]] = 0
        # (1) v_r, (2) v_t
        for destroy_no in destroy_nos:
            del self.vehicle_routes[destroy_no]
            del self.vehicle_time[destroy_no]

    def update_from_random_repair(self, rnd_state: rnd.RandomState):
        # (1) get space routes
        repair_space_route = self.get_random_SpaceRoutes(rnd_state=rnd_state)

        # (2) from space routes to calculate TS routes
        # repair_TS_route = self.get_TSRoutes_from_SpaceRoutes_single(repair_space_route)
        repair_TS_route = self.get_TSRoutes_from_SpaceRoutes_global(repair_space_route=repair_space_route)
        return repair_TS_route
    
    def get_random_SpaceRoutes(self, rnd_state:rnd.RandomState):
        repair_space_route = defaultdict(list)
        for id in self.destroy_nos:
            des = self.destinations[id]
            start_loc = self.start_location[id]
            current_space_point = start_loc[0]
            neighbour_point = utils.get_neighbour_point(current_space_point, self.bi_space_arc)
            while des not in neighbour_point:
                # seleted_point = np.random.choice(neighbour_point, 1)
                seleted_point = rnd_state.choice(neighbour_point, 1)
                repair_space_route[id].append((int(current_space_point), int(seleted_point)))
                current_space_point = int(seleted_point)
                neighbour_point = utils.get_neighbour_point(current_space_point, self.bi_space_arc)
            repair_space_route[id].append((int(current_space_point), des))
        return repair_space_route

    def get_TSRoutes_from_SpaceRoutes_single(self, repair_space_route: dict):     # dict(list(tuple))
        new_TS_routes = defaultdict(list) # defaultdict is not easy for comerge
        for k, spc_arcs in repair_space_route.items():
            # for each repaired vehicle route, usual only one vehicle destroyed one step
            start_time = self.start_location[k][1]
            for spc_arc in spc_arcs:
                TS_arc, arrival_time = self.calculateTime_from_BlockNo_single(spc_arc, start_time)
                self.vehicle_time[k] += arrival_time
                start_time += arrival_time
                new_TS_routes[k].append(TS_arc)
            self.vehicle_routes[k] = new_TS_routes[k]
        return new_TS_routes
    
    def get_TSRoutes_from_SpaceRoutes_global(self, repair_space_route: defaultdict(list)):
        # (1) get all vehicle space routes
        for v_id, TS_arcs in self.vehicle_routes.items():
            for arc in TS_arcs:
                repair_space_route[v_id].append((arc[0][0],arc[1][0]))
        
        # (2) get TS route from all space route
        # 2.0 init 
        task_finish = defaultdict(int)
        new_TS_routes = defaultdict(list)
        new_road_traffic = defaultdict(lambda: defaultdict(int))
        new_point_traffic = defaultdict(int)  
        new_vehicle_time = defaultdict(int)                       
        new_road_block = defaultdict(lambda: defaultdict(int))   
        new_point_block = defaultdict(int)
        length_stat = defaultdict(int)
        # two pointer to indicate current process for each vehicle
        arc_index = defaultdict(int)    # save each vehicle arc index
        prev_TS_point = defaultdict(tuple) # save previous time-space node: (space, time)
        for v_id in repair_space_route.keys():
            task_finish[v_id] = 0
            prev_TS_point[v_id] = self.start_location[v_id]
        timestep = 0
        total_sum = sum(value for value in task_finish.values())

        # start iteration
        while total_sum != len(task_finish):
            tmp_point_index = []     # used for point block
            active_v_ids = [v_id for v_id, value in task_finish.items() if value == 0]
            # 2.1 first count road vehicle number to get global block situation
            for v_id in active_v_ids:
                tmp_road = repair_space_route[v_id][arc_index[v_id]]
                new_road_traffic[tmp_road][timestep] += 1
                
            # 2.2 record route length
            for v_id in active_v_ids:
                tmp_road = repair_space_route[v_id][arc_index[v_id]]
                # block or not
                if new_road_traffic[tmp_road][timestep] > self.b0:
                    new_road_block[tmp_road][timestep] = 1
                    length_stat[v_id] += self.vehicle_speed / 2 
                else:
                    assert new_road_block[tmp_road][timestep] == 0, 'error block situation'
                    length_stat[v_id] += self.vehicle_speed 

                # arrived next point
                if length_stat[v_id] >= self.space_distance[tmp_road]: 
                    arc_index[v_id] += 1
                    arrived_TS_point = (tmp_road[1], timestep+1)
                    new_point_traffic[arrived_TS_point] += 1
                    new_TS_routes[v_id].append((prev_TS_point[v_id], arrived_TS_point))
                    prev_TS_point[v_id] = arrived_TS_point
                    tmp_point_index.append(arrived_TS_point)
                # task finished
                if arc_index[v_id] == len(repair_space_route[v_id]):
                    task_finish[v_id] = 1
                    new_vehicle_time[v_id] += arrived_TS_point[1] - self.start_location[v_id][1] # final point time - first point time
            
            for index in tmp_point_index:
                if new_point_traffic[index] > self.p0:
                    new_point_block[index] = 1
            timestep += 1
            total_sum = sum(value for value in task_finish.values())

        # all task finished
        # give the value
        self.update_from_parameter(v_r=new_TS_routes, v_t=new_vehicle_time, r_t=new_road_traffic, r_b=new_road_block,
                                   p_t=new_point_traffic, p_b=new_point_block)
        return new_TS_routes
    
    def update_from_parameter(self, v_r: defaultdict(list), v_t, r_t, p_t, r_b, p_b):
        # cal by using hash to reduce the time, so it is diffence with the init update
        self.vehicle_routes = v_r
        self.vehicle_time = v_t
        self.road_traffic = r_t
        self.point_traffic = p_t
        self.road_block = r_b
        self.point_block = p_b
        
    def get_bidirection_arc(self):
        add_arc = []
        for arc in self.space_arc:
            add_arc.append((arc[1], arc[0]))
        return self.space_arc + add_arc 

    def get_context(self):
        # TODO implement a method returning a context vector. This is only
        #  needed for some context-aware bandit selectors from MABWiser;
        #  if you do not use those, this default is already sufficient!
        return None
    
    def render_info(self):
        print(f'T:{self.T}, B:{self.B}, P:{self.P}')

    def calculateTime_from_BlockNo_single(self, spc_arc, start_time):
        '''
        this step need to sort all vehicle routes to slow down if change to block
        if wanted to reduce the calculate time, 
        (1) using hash to save d = {'road_id':[v_ids]}, still remain a lot calculation
        (2) using new destroy and repair, destroy not obey the block rules road ids, only repair the obviation road ids, using large penalty
        '''
        length = 0
        tmp_t = 0
        while length < self.space_distance[spc_arc]:
            self.road_traffic[spc_arc][start_time + tmp_t] += 1
            # (1) if change to block(b=b0), all vehicle need to slow down
            if self.road_traffic[spc_arc][start_time + tmp_t] == self.b0:
                v_spd = self.vehicle_speed / 2
                self. road_block[spc_arc][start_time + tmp_t] = 1
            # （2） if already block (b>b0)
            elif self.road_block[spc_arc][start_time + tmp_t] == 1:
                pass
            else:
                v_spd = self.vehicle_speed
            length += v_spd
            tmp_t += 1
        
        return ((spc_arc[0], start_time), (spc_arc[1], start_time + tmp_t)), tmp_t

    def calculateTime_from_BlockNo_global(self, spc_arc):
        # get all space arc
        pass