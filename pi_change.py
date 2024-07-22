import torch
from torch.distributions import Categorical

def convert_prob_to_map(pi, target):
    """
    将概率矩阵转换为地图上的概率表示。

    参数:
    - pi: 形状为 (batch_size, num_agents, target_n) 的概率矩阵
    - target: 形状为 (batch_size, map_size_x, map_size_y) 的目标点矩阵，目标点位置为1，其它位置为0

    返回:
    - 形状为 (batch_size, num_agents, map_size_x, map_size_y) 的概率地图表示
    """
    batch_size, num_agents, target_n = pi.shape
    _, map_size_x, map_size_y = target.shape

    # 初始化一个全零矩阵，形状为 (batch_size, num_agents, map_size_x, map_size_y)
    prob_map = torch.zeros((batch_size, num_agents, map_size_x, map_size_y), device=pi.device)

    for b in range(batch_size):
        # 找到目标点的坐标
        target_coords = [(i, j) for i in range(map_size_x) for j in range(map_size_y) if target[b, i, j] == 1]

        # 确保目标点数量和 target_n 一致
        assert len(target_coords) == target_n, f"目标点数量 {len(target_coords)} 与 target_n {target_n} 不一致"

        for t, (x, y) in enumerate(target_coords):
            for a in range(num_agents):
                prob_map[b, a, x, y] = pi[b, a, t]

    return prob_map

def action_sample(pi):
    dist = Categorical(pi.transpose(2, 1))
    action = dist.sample()
    log_prob = dist.log_prob(action)
    return action, log_prob


if __name__ == '__main__':
    # 示例数据
    pi = torch.tensor([[[0.2556, 0.2562, 0.2382, 0.2500],
                    [0.2493, 0.2391, 0.2573, 0.2544],
                    [0.2562, 0.2301, 0.2620, 0.2517]],

                   [[0.2615, 0.2466, 0.2519, 0.2399],
                    [0.2566, 0.2462, 0.2498, 0.2474],
                    [0.2474, 0.2269, 0.2667, 0.2590]]], device='cuda:0')

    target = torch.tensor([[[0., 1., 0., 0., 0.],
                        [0., 1., 0., 0., 0.],
                        [0., 0., 0., 0., 0.],
                        [0., 1., 0., 0., 0.],
                        [0., 1., 0., 0., 0.]],

                       [[0., 0., 0., 1., 0.],
                        [0., 0., 0., 0., 1.],
                        [0., 0., 0., 0., 0.],
                        [0., 1., 0., 0., 0.],
                        [0., 0., 0., 1., 0.]]], device='cuda:0')

    # 转换并输出结果
    prob_map = convert_prob_to_map(pi, target)
    print(prob_map)
