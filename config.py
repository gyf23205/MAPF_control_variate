import torch
import numpy as np
from coord_transform import random_constraints

target_n = 4
obstacle_n = 2
agent_n = 2
destination_n = agent_n
start_n = agent_n
map_size_x = 5
map_size_y = 5
batchsize = 2

'''
    target_constraints = {
        0: [2,3],  # 第一个目标点只能被代理1和7访问
        1: [1,2],  # 第二个目标点只能被代理3和5访问
        2: [0],
        3: [1,2,3]
    }
'''
if __name__ == '__main__':
    target_constraints = random_constraints(target_n, agent_n)
#print(target_constraints)