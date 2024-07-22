import subprocess
import os
import torch
import json
from data_generator import map_size_x, map_size_y, agent_n, batchsize

from multiprocessing import Pool
from config import target_n

from getpath_new_allocation import PathPlanningCNN, load_data, my_allocation_method
from pi_change import convert_prob_to_map, action_sample

def tensor_to_text(tensor):
    layers = []
    for layer in tensor:
        text = '\n'.join(''.join('T' if x == 1 else '.' for x in row) for row in layer)
        layers.append(text)
    return layers

def create_and_write_file(file_path, header, content):
    full_content = f"{header}{content}"
    try:
        with open(file_path, 'w') as f:
            f.write(full_content)
        #print(f"成功创建并写入文件: {file_path}")
    except IOError as e:
        print(f"创建并写入文件时出错: {e}")
        pass

def convert_to_index(x, y, width):
    return y * width + x

def extract_points(tensor, map_size_x, map_size_y):
    batched_points = {i: [] for i in range(agent_n)}
    start_points = {i: [] for i in range(agent_n)}

    for i in range(tensor.size(0)):
        current_points = {j: [] for j in range(agent_n)}
        for j in range(tensor.size(1)):
            for k in range(tensor.size(2)):
                val = tensor[i, j, k].item()
                if val != -1:
                    index = convert_to_index(k, j, map_size_y)
                    current_points[val].append(index)
                    if i == 0 and start_points[val] == []:
                        start_points[val].append(index)
        for agent in current_points:
            batched_points[agent].append(current_points[agent])
            try:
                if i > 0:
                    start_points[agent].append(current_points[agent][0])
            except:
                pass




    return batched_points, start_points

def ensure_directory_exists(directory):
    os.makedirs(directory, exist_ok=True)

def generate_json_file(params):
    agent, batch_index, start_points, target_points, base_file_path = params
    agent_start_point = start_points[agent][batch_index]
    agent_target_points = target_points[agent][batch_index]

    data = {
        "data": [
            {
                "source": agent_start_point,
                "targetSet": agent_target_points,
                "node_constraints": {}
            }
        ]
    }
    filename = f"{base_file_path}/input_agent{agent}_batch{batch_index}.json"
    file_content = json.dumps(data, indent=4)
    try:
        with open(filename, 'w') as f:
            f.write(file_content)
        print(f"成功生成文件: {filename}")
    except IOError as e:
        print(f"生成文件时出错: {e}")
        pass

def run_wsl_command(command):
    wsl_path = r"C:\Windows\System32\wsl.exe"
    path = "/mnt/c/ubuntu/tsp_solver/cpp-do/build"
    full_command = f"cd {path} && {command}"
    try:
        completed_process = subprocess.run([wsl_path, "bash", "-c", full_command], text=True, capture_output=True, encoding='utf-8')
        print(f"运行命令: {full_command}")
        print("STDOUT:", completed_process.stdout)
        print("STDERR:", completed_process.stderr)
    except Exception as e:
        print("Error executing command:", e)
        pass

def extract_cost_from_output(output_file_path):
    costs = []
    try:
        with open(output_file_path, 'r') as file:
            for line in file:
                if "cost" in line:
                    cost_string = line.split("[")[1].split(",")[0]
                    cost = int(cost_string)
                    costs.append(cost)
        return costs
    except FileNotFoundError:
        print(f"文件 '{output_file_path}' 不存在")
        pass
    except Exception as e:
        print(f"提取 cost 时出错: {e}")
        pass

def tsp_solver(targets, obstacles, starts, result, start_map, pi):
    texts = tensor_to_text(obstacles)
    base_file_path = "C:/ubuntu/tsp_solver/cpp-do/test/map/output_map_"
    header = f"type octile\nheight {map_size_x}\nwidth {map_size_y}\nmap\n"

    ensure_directory_exists("C:/ubuntu/tsp_solver/cpp-do/test/map")

    for i, text in enumerate(texts, 1):
        file_path = f"{base_file_path}{i}.map"
        create_and_write_file(file_path, header, text)

    all_points, start_points = extract_points(start_map, map_size_x, map_size_y)
    target_points, p = extract_points(result, map_size_x, map_size_y)
    #print(start_points)
    #print(target_points)

    base_file_path = "C:/ubuntu/tsp_solver/cpp-do/test/map_input"
    ensure_directory_exists(base_file_path)

    params = [(agent, batch_index, start_points, target_points, base_file_path)
              for batch_index in range(len(target_points[0]))
              for agent in range(agent_n)]

    with Pool() as pool:
        pool.map(generate_json_file, params)

    actualcost = []
    commands = [
        f"./api ../test/map/output_map_{i + 1}.map ../test/map_input/input_agent{j}_batch{i}.json ../test/output/ans_batch{i}_agent{j}.txt"
        for i in range(batchsize)
        for j in range(agent_n)
    ]

    with Pool() as pool:
        pool.map(run_wsl_command, commands)

    for i in range(batchsize):
        total_cost = 0
        for j in range(agent_n):
            output_file_path = f'C:/ubuntu/tsp_solver/cpp-do/test/output/ans_batch{i}_agent{j}.txt'
            costs = extract_cost_from_output(output_file_path)
            if costs:
                total_cost += costs[0]
        actualcost.append(total_cost)

    actualcost_tensor = torch.tensor(actualcost).view(-1, 1)

    actualcost_tensor -= target_n  #去掉求解器中serve和wait时间

    #actualcost_tensor = actualcost_tensor/50

    return actualcost_tensor

if __name__ == '__main__':

    #targets = load_data('./validation_data/validation_data_targets.pt').to('cpu')
    #obstacles = load_data('./validation_data/validation_data_obstacles.pt').to('cpu')
    #destinations = load_data('./validation_data/validation_data_destinations.pt').to('cpu')
    #starts = load_data('./validation_data/validation_data_starts.pt').to('cpu')
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    data_idx=0
    targets = load_data(f'./training_data/training_data_targets_{data_idx}.pt').to(device)
    obstacles = load_data(f'./training_data/training_data_obstacles_{data_idx}.pt').to(device)
    destinations = load_data(f'./training_data/training_data_destinations_{data_idx}.pt').to(device)
    starts = load_data(f'./training_data/training_data_starts_{data_idx}.pt').to(device)


    #print(obstacles)
    #print(tensor_to_text(obstacles))

    #print(starts)

    #print(targets)

    cnn_model = PathPlanningCNN().to(device)
    inputs_cnn = torch.stack([targets, obstacles, starts], dim=1).to(device)
    pi = cnn_model(inputs_cnn)
    #print(pi)
    pi_transform = convert_prob_to_map(pi, targets)
    action, log_probs = action_sample(pi)

    allocation, start_map = my_allocation_method(pi_transform, targets, obstacles,
                                                                  starts, device)

# path, result, start_map, destination_map, pi = my_allocation_method(pi,targets, obstacles, destinations, starts)
    #print(allocation)
    actual_costs = tsp_solver(targets, obstacles,  starts, allocation, start_map,  pi_transform)

    #print("实际成本张量:", actual_costs)
    #print("实际成本列表:", actual_costs.tolist())
