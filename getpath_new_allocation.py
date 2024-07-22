import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, TensorDataset
from data_generator import generate_and_save_data
from config import agent_n,map_size_x,map_size_y,target_n
import torch.nn.functional as F
import copy
from data_generator import agent_n,batchsize
#from data_generator import target_constraints
from coord_transform import assign_indices,find_coordinates,generate_map_matrices
from coord_transform import constrained_allocation




class PathPlanningCNN(nn.Module):
    def __init__(self):
        super(PathPlanningCNN, self).__init__()
        self.conv1 = nn.Conv2d(3, 64, kernel_size=3, stride=1, padding=1)
        self.conv2 = nn.Conv2d(64, 128, kernel_size=3, stride=1, padding=1)
        self.fc1 = nn.Linear(128 * map_size_x*map_size_y,  agent_n * target_n)

    def forward(self, x):
        x = F.relu(self.conv1(x))
        x = F.relu(self.conv2(x))
        x = x.view(x.size(0), -1)
        x = self.fc1(x)
        x = x.view(-1, agent_n, target_n)
        #x = F.softmax(x, dim=2)  # Apply softmax along the agent dimension
        x = F.softmax(x, dim=1)  # Apply softmax along the target dimension
        return x

def forward(self, x):
        x = F.relu(self.conv1(x))
        x = F.relu(self.conv2(x))
        x = F.relu(self.conv3(x))

        # 将特征图展平为一维
        x = x.view(x.size(0), -1)

        # 应用全连接层
        x = F.relu(self.fc1(x))
        x = self.fc2(x)

        # 将输出重新调整为卷积层的输出形状
        x = x.view(x.size(0), -1, map_size_x, map_size_y)

        return self.softmax(x)  # 应用 softmax


def load_data(file_path):
    data = torch.load(file_path)
    return data



def my_allocation_method(pi, targets, obstacles, starts, device):
    # 打印 pi 矩阵
    print("pi:")
    print(pi)

    # 找到每个 agent 概率最大的目标点的索引
    max_indices = torch.argmax(pi, dim=1)
    #print("max_indices (before masking):")
    #print(max_indices)

    # 将概率为 0 的点设为 -1
    zero_mask = pi.max(dim=1).values == 0
    max_indices = max_indices.masked_fill(zero_mask, -1)
    #print("max_indices (after masking):")
    #print(max_indices)

    starts_map = assign_indices(starts.to(device), agent_n)

    #print("starts_map:")
    #print(starts_map)

    return max_indices, starts_map


'''
def replace_non_negative(tensor, allocation):
    for batch_idx in range(tensor.size(0)):  # 遍历batch维度
        biaozhi=0
        for i in range(tensor.size(1)):  # 遍历空间维度的第一维
            for j in range(tensor.size(2)):  # 遍历空间维度的第二维
                # 检查tensor中的值是否不等于-1
                if tensor[batch_idx, i, j] != -1 and biaozhi==0:
                    # 将tensor中的非-1值复制到allocation的对应位置
                    allocation[batch_idx, i, j] = tensor[batch_idx, i, j]
                    biaozhi=1

    return allocation
'''
