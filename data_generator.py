import torch
import numpy as np
from config import *

def generate_targets_obstacles_and_destinations(target_n, obstacle_n, destination_n, start_n, map_size_x, map_size_y):
    targets_map = torch.zeros((map_size_x, map_size_y))
    obstacles_map = torch.zeros((map_size_x, map_size_y))
    destinations_map = torch.zeros((map_size_x, map_size_y))
    starts_map = torch.zeros((map_size_x, map_size_y))  # New starts map

    # Generate targets
    for _ in range(target_n):
        while True:
            x, y = np.random.randint(0, map_size_x), np.random.randint(0, map_size_y)
            if targets_map[x, y] == 0:
                targets_map[x, y] = 1
                break

    # Generate obstacles
    for _ in range(obstacle_n):
        while True:
            x, y = np.random.randint(0, map_size_x), np.random.randint(0, map_size_y)
            if obstacles_map[x, y] == 0 and targets_map[x, y] == 0:
                obstacles_map[x, y] = 1
                break

    # Generate destinations
    for _ in range(destination_n):
        while True:
            x, y = np.random.randint(0, map_size_x), np.random.randint(0, map_size_y)
            if destinations_map[x, y] == 0 and targets_map[x, y] == 0 and obstacles_map[x, y] == 0:
                destinations_map[x, y] = 1
                break

    # Generate start points
    for _ in range(start_n):
        while True:
            x, y = np.random.randint(0, map_size_x), np.random.randint(0, map_size_y)
            if starts_map[x, y] == 0 and targets_map[x, y] == 0 and obstacles_map[x, y] == 0 and destinations_map[x, y] == 0:
                starts_map[x, y] = 1
                break

    return targets_map, obstacles_map, destinations_map, starts_map

def generate_and_save_data(target_n, obstacle_n, destination_n, start_n, map_size_x, map_size_y, batchsize, folder_path):
    targets_map_list = []
    obstacles_map_list = []
    destinations_map_list = []
    starts_map_list = []

    for _ in range(batchsize):
        targets_map, obstacles_map, destinations_map, starts_map = generate_targets_obstacles_and_destinations(target_n, obstacle_n, destination_n, start_n, map_size_x, map_size_y)
        targets_map_list.append(targets_map)
        obstacles_map_list.append(obstacles_map)
        destinations_map_list.append(destinations_map)
        starts_map_list.append(starts_map)

    targets_mapping = torch.stack(targets_map_list)
    obstacles_mapping = torch.stack(obstacles_map_list)
    destinations_mapping = torch.stack(destinations_map_list)
    starts_mapping = torch.stack(starts_map_list)

    print(targets_mapping)
    print(obstacles_mapping)
    print(destinations_mapping)
    print(starts_mapping)


    torch.save(targets_mapping, f'{folder_path}/{folder_path}_targets.pt')
    torch.save(obstacles_mapping, f'{folder_path}/{folder_path}_obstacles.pt')
    torch.save(destinations_mapping, f'{folder_path}/{folder_path}_destinations.pt')
    torch.save(starts_mapping, f'{folder_path}/{folder_path}_starts.pt')

def generate_and_save_data_train(target_n, obstacle_n, destination_n, start_n, map_size_x, map_size_y, batchsize, folder_path):
    for i in range(10):
        targets_map_list = []
        obstacles_map_list = []
        destinations_map_list = []
        starts_map_list = []

        for _ in range(batchsize):
            targets_map, obstacles_map, destinations_map, starts_map = generate_targets_obstacles_and_destinations(target_n, obstacle_n, destination_n, start_n, map_size_x, map_size_y)
            targets_map_list.append(targets_map)
            obstacles_map_list.append(obstacles_map)
            destinations_map_list.append(destinations_map)
            starts_map_list.append(starts_map)

        targets_mapping = torch.stack(targets_map_list)
        obstacles_mapping = torch.stack(obstacles_map_list)
        destinations_mapping = torch.stack(destinations_map_list)
        starts_mapping = torch.stack(starts_map_list)

        #print(targets_mapping)
        #print(obstacles_mapping)
        #print(destinations_mapping)
        #print(starts_mapping)

        torch.save(targets_mapping, f'{folder_path}/{folder_path}_targets_{i}.pt')
        torch.save(obstacles_mapping, f'{folder_path}/{folder_path}_obstacles_{i}.pt')
        torch.save(destinations_mapping, f'{folder_path}/{folder_path}_destinations_{i}.pt')
        torch.save(starts_mapping, f'{folder_path}/{folder_path}_starts_{i}.pt')

def main():
    # Set random seeds
    torch.manual_seed(127)
    np.random.seed(4233)


    target_constraints = random_constraints(target_n, agent_n)
    #print(target_constraints)

    # Example usage
    generate_and_save_data(target_n, obstacle_n, destination_n, start_n, map_size_x, map_size_y, batchsize, './validation_data')
    generate_and_save_data_train(target_n, obstacle_n, destination_n, start_n, map_size_x, map_size_y, batchsize, './training_data')
    #generate_and_save_data(target_n, obstacle_n, destination_n, start_n, map_size_x, map_size_y, batchsize, './testing_data')


if __name__ == "__main__":
    main()
