import torch

def my_allocation_method(pi, device):
    # 打印 pi 矩阵
    print("pi:")
    print(pi)

    # 找到每个 agent 概率最大的目标点的索引
    max_indices = torch.argmax(pi, dim=1)
    print("max_indices (before masking):")
    print(max_indices)

    # 将概率为 0 的点设为 -1
    zero_mask = pi.max(dim=1).values == 0
    max_indices = max_indices.masked_fill(zero_mask, -1)
    print("max_indices (after masking):")
    print(max_indices)

    return max_indices


pi = torch.tensor([[[[0.2108, 0.0000, 0.0000, 0.0000],
                     [0.0000, 0.2096, 0.0000, 0.1898],
                     [0.0000, 0.0000, 0.0000, 0.1942],
                     [0.0000, 0.0000, 0.0000, 0.1956]],

                    [[0.2008, 0.0000, 0.0000, 0.0000],
                     [0.0000, 0.2090, 0.0000, 0.1982],
                     [0.0000, 0.0000, 0.0000, 0.2043],
                     [0.0000, 0.0000, 0.0000, 0.1876]],

                    [[0.2104, 0.0000, 0.0000, 0.0000],
                     [0.0000, 0.1961, 0.0000, 0.1981],
                     [0.0000, 0.0000, 0.0000, 0.2029],
                     [0.0000, 0.0000, 0.0000, 0.1925]]],


                   [[[0.0000, 0.0000, 0.0000, 0.0000],
                     [0.0000, 0.0000, 0.2126, 0.2059],
                     [0.0000, 0.1913, 0.1911, 0.0000],
                     [0.0000, 0.1991, 0.0000, 0.0000]],

                    [[0.0000, 0.0000, 0.0000, 0.0000],
                     [0.0000, 0.0000, 0.2029, 0.2040],
                     [0.0000, 0.1967, 0.2019, 0.0000],
                     [0.0000, 0.1945, 0.0000, 0.0000]],

                    [[0.0000, 0.0000, 0.0000, 0.0000],
                     [0.0000, 0.0000, 0.2040, 0.1982],
                     [0.0000, 0.1961, 0.2020, 0.0000],
                     [0.0000, 0.1997, 0.0000, 0.0000]]]], device='cuda:0')


device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')


max_indices = my_allocation_method(pi, device)
print("输出:")
print(max_indices)
