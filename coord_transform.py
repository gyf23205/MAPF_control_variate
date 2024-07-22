import torch
import numpy as np

def random_constraints(num_targets, max_agent_index):

    target_constraints = {}
    for target in range(num_targets):
        # Ensure at least one agent per target, up to all agents
        num_agents = np.random.randint(1, max_agent_index + 2)
        agents = np.random.choice(range(max_agent_index + 1), size=num_agents, replace=False)
        target_constraints[target] = agents.tolist()

    return target_constraints

def coordinate_transform(input_tensor):
    # 确认输入是三维的PyTorch张量
    if input_tensor.dim() != 3:
        raise ValueError("Expected a 3-dimensional tensor")

    # 选择张量的第一个元素（假设目标点定义在这里）
    # 并找到所有目标点的坐标
    target_coords = (input_tensor[0] == 1).nonzero(as_tuple=False)

    return target_coords

def assign_indices(tensor, num_agents):
    batch_size, map_size_x, map_size_y = tensor.shape
    result = torch.full_like(tensor, -1)  # 初始化结果张量，所有值设为-1

    for b in range(batch_size):  # 遍历每个批次
        ones_indices = (tensor[b] == 1).nonzero(as_tuple=True)  # 找出当前批次中所有1的位置
        num_ones = len(ones_indices[0])

        if num_ones > num_agents:
            print(f"Warning: The number of ones ({num_ones}) in the tensor exceeds the number of agents ({num_agents}).")
            num_ones = num_agents

        # 生成0到num_agents-1的不重复数字序列
        shuffled_indices = torch.randperm(num_agents)[:num_ones]

        # 为当前批次中1的位置分配索引
        for i in range(num_ones):
            x, y = ones_indices[0][i], ones_indices[1][i]
            result[b, x, y] = shuffled_indices[i]

    return result

def constrained_allocation(softmax_output, constraints, target_coords):
    # softmax_output: [batch_size, num_agents, x_coord, y_coord]
    # constraints: {target_index: [valid_agents]}
    # target_coords: [(x1, y1), (x2, y2), ...] 目标点的坐标列表

    batch_size = softmax_output.shape[0]
    num_agents = softmax_output.shape[1]

    # 初始化分配矩阵，初始值为-1表示未分配
    allocation = torch.full((batch_size, num_agents, 2), -1)

    for batch_idx in range(batch_size):
        for target_idx, (x, y) in enumerate(target_coords):
            # 获取符合条件的代理列表
            valid_agents = constraints[target_idx]

            # 提取对应目标点的所有代理的softmax概率
            probs = softmax_output[batch_idx, valid_agents, x, y]

            # 过滤出符合约束的代理的概率，并找到最大值
            max_prob, max_idx = torch.max(probs, dim=0)

            # 分配目标点给概率最高的符合条件的代理
            if max_prob > 0:  # 确保有有效的概率值
                # 分配坐标而非索引
                allocation[batch_idx, valid_agents[max_idx]] = torch.tensor([x, y])

    return allocation

def find_coordinates(tensor):
    """
    根据给定的张量，找到值为1的元素的坐标。
    参数:
        tensor: 一个三维的张量，其中第一维表示批次。
    返回:
        一个列表，包含每个批次的坐标列表，每个坐标也以列表形式表示。
    """
    # 存储每个批次坐标的列表
    batch_coordinates = []

    # 遍历张量以找到值为1的元素的坐标
    for batch_idx, batch in enumerate(tensor):
        # 存储当前批次坐标的列表
        current_batch_coords = []

        for row_idx, row in enumerate(batch):
            for col_idx, element in enumerate(row):
                if element == 1:
                    # 使用列表存储行索引和列索引
                    current_batch_coords.append([row_idx, col_idx])

        # 将当前批次的坐标列表添加到总列表中
        batch_coordinates.append(current_batch_coords)

    return batch_coordinates

def generate_map_matrices(softmax_results, batch_target_points, agent_selection_rules):
    # 地图大小
    map_height, map_width = softmax_results.shape[2], softmax_results.shape[3]

    batch_map_matrices = []  # 存储每个批次地图矩阵的列表

    for batch_idx, target_points in enumerate(batch_target_points):
        # 使用嵌套列表创建地图矩阵，初始值为-1
        map_matrix = [[-1 for _ in range(map_width)] for _ in range(map_height)]

        for idx, (x, y) in enumerate(target_points):
            agents = agent_selection_rules.get(idx % len(agent_selection_rules), [])
            valid_agents = [agent for agent in agents if agent < softmax_results.size(1)]

            if not valid_agents:
                continue

            # 获取这些代理在当前目标点坐标下的概率值
            probs = softmax_results[batch_idx, valid_agents, x, y]

            # 找到概率最大的代理的索引
            max_prob_agent_idx = torch.argmax(probs)

            # 确保max_prob_agent的数据类型为torch.int32
            # 由于max_prob_agent_idx是一个索引，我们直接从valid_agents列表中获取整数值
            max_prob_agent = valid_agents[max_prob_agent_idx]

            # 在map_matrix中赋值，之后将转换为torch.Tensor
            map_matrix[x][y] = max_prob_agent

        # 将当前批次的地图矩阵转换为torch.Tensor并添加到列表中
        # 这里确保转换后的torch.Tensor的数据类型为torch.int32
        batch_map_matrices.append(torch.tensor(map_matrix, dtype=torch.int32))

    # 使用torch.stack()将这些张量沿着一个新的批次维度堆叠起来
    stacked_tensor = torch.stack(batch_map_matrices, dim=0)

    return stacked_tensor



if __name__ == "__main__":

    # 示例使用
    # 假设有一个三维的PyTorch张量
    input_tensor = torch.tensor([
        [[0, 0, 1, 0],
        [0, 1, 0, 0],
        [0, 0, 0, 1],
        [1, 0, 0, 0]]
    ])

    # 使用修改后的coordinate_transform函数
    coords = coordinate_transform(input_tensor)
    #print(coords)
