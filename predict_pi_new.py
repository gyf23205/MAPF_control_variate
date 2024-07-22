import torch
import torch.nn as nn
import torch
from torch.distributions import Categorical


def action_sample(pi):
    """
    从形状为 (batch_size, num_agents, x, y) 的张量 pi 中采样动作，并返回采样结果和对数概率。

    参数:
    pi (torch.Tensor): 概率张量，形状为 (batch_size, num_agents, x, y)。

    返回:
    tuple: 包含采样后的动作和对数概率的元组。
    """
    # 将维度转置为 (batch_size, x*y, num_agents) 以适应 Categorical 的输入格式
    pi_reshaped = pi.view(pi.size(0), pi.size(1), -1).transpose(2, 1)

    # 创建 Categorical 分布对象
    dist = Categorical(pi_reshaped)

    # 进行采样
    action = dist.sample()

    # 计算对数概率
    log_prob = dist.log_prob(action)

    return action, log_prob


class SurrogateNetwork(nn.Module):
    def __init__(self, in_dim: int, out_dim: int, n_hidden: int = 64, nonlin: str = 'tanh', dev='cpu', **kwargs):
        super(SurrogateNetwork, self).__init__()
        # 定义激活函数列表
        nlist = {
            'relu': nn.ReLU(),
            'tanh': nn.Tanh(),
            'sigmoid': nn.Sigmoid(),
            'softplus': nn.Softplus(),
            'lrelu': nn.LeakyReLU(),
            'elu': nn.ELU()
        }
        # 定义网络层
        self.layer = nn.Linear(in_dim, n_hidden, device=dev)
        self.layer2 = nn.Linear(n_hidden, n_hidden, device=dev)
        self.out = nn.Linear(n_hidden, out_dim, device=dev)
        self.nonlin = nlist[nonlin]

    def forward(self, x, **kwargs):
        x = self.layer(x)
        x = self.nonlin(x)
        x = self.layer2(x)
        x = self.nonlin(x)
        x = self.out(x)
        return x
