import torch
import torch.nn as nn
import torch.optim as optim
import wandb
import torch.nn.functional as F

from config import agent_n, batchsize, map_size_x, map_size_y, target_n
from solver_windows import tsp_solver
from getpath_new_allocation import PathPlanningCNN, load_data, my_allocation_method
from predict_pi_new import SurrogateNetwork
from pi_change import convert_prob_to_map, action_sample

# torch.autograd.set_detect_anomaly(True)

def validate_model(cnn_model, surrogate_model, validation_targets, validation_obstacles, validation_starts, device):
    cnn_model.eval()
    surrogate_model.eval()
    with torch.no_grad():
        targets = validation_targets.to(device)
        obstacles = validation_obstacles.to(device)
        starts = validation_starts.to(device)

        inputs_cnn = torch.stack([targets, obstacles, starts], dim=1).to(device)
        pi = cnn_model(inputs_cnn)
        pi = convert_prob_to_map(pi, targets)

        allocation, start_map = my_allocation_method(pi, targets, obstacles, starts, device)

        actual_costs = tsp_solver(targets, obstacles, starts, allocation, start_map, pi)
        total_validation_cost = actual_costs.sum().item()

    cnn_model.train()
    surrogate_model.train()
    return total_validation_cost


def main():
    wandb.login(key='')
    wandb.init(project='mmtsp', config={
        'num_epochs': 1500,
        'batch_size': batchsize,
        'learning_rate': 0.001
    })

    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

    # 加载验证数据
    validation_targets = load_data('./validation_data/validation_data_targets.pt').to(device)
    validation_obstacles = load_data('./validation_data/validation_data_obstacles.pt').to(device)
    validation_destinations = load_data('./validation_data/validation_data_destinations.pt').to(device)
    validation_starts = load_data("./validation_data/validation_data_starts.pt").to(device)

    record = []
    loss_record = []
    cnn_model = PathPlanningCNN().to(device)


    surrogate_model = SurrogateNetwork(in_dim=target_n, out_dim=1, n_hidden=256, nonlin='tanh', dev=device).to(device)

    cnn_optimizer = optim.RMSprop(cnn_model.parameters(), lr=1e-4, momentum=0.468, weight_decay=0.067)
    surrogate_optimizer = optim.RMSprop(surrogate_model.parameters(), lr=1e-3, momentum=0.202, weight_decay=0.336)

    scheduler_p = torch.optim.lr_scheduler.ReduceLROnPlateau(cnn_optimizer, min_lr=1e-7, patience=50, factor=0.5, verbose=True)
    scheduler_s = torch.optim.lr_scheduler.ReduceLROnPlateau(surrogate_optimizer, min_lr=1e-7, patience=50, factor=0.5, verbose=True)

    num_epochs = 1500
    best_so_far = float('inf')
    total_batches_logged = 0

    for epoch in range(num_epochs):
        total_actual_cost = 0
        total_epoch_loss = 0

        data_idx = epoch % 10
        targets = load_data(f'./training_data/training_data_targets_{data_idx}.pt').to(device)
        obstacles = load_data(f'./training_data/training_data_obstacles_{data_idx}.pt').to(device)
        #destinations = load_data(f'./training_data/training_data_destinations_{data_idx}.pt').to(device)
        starts = load_data(f'./training_data/training_data_starts_{data_idx}.pt').to(device)

        targets = targets.to(device)
        obstacles = obstacles.to(device)
        #destinations = destinations.to(device)
        starts = starts.to(device)

        inputs_cnn = torch.stack([targets, obstacles, starts], dim=1).to(device)
        pi = cnn_model(inputs_cnn)
        pi_transform = convert_prob_to_map(pi, targets)
        action, log_probs = action_sample(pi)

        allocation, start_map = my_allocation_method(pi_transform, targets, obstacles,  starts, device)

        #reshaped_probs = pi_transform.view(pi_transform.size(0), -1).float().to(device)
        #log_probs_predict = torch.log(reshaped_probs + 1e-15)  # 避免 log(0)

        predicted_costs = torch.squeeze(surrogate_model(log_probs))

        actual_costs = tsp_solver(targets, obstacles,  starts, allocation, start_map,  pi_transform)
        actual_costs_tensor = torch.tensor(actual_costs, dtype=torch.float32).to(device).detach()

        #predicted_costs = predicted_costs.unsqueeze(1)
        actual_costs_tensor = actual_costs_tensor.view(-1)

        cost_difference = actual_costs_tensor - predicted_costs.detach()

        print(actual_costs_tensor)
        print(log_probs)

        #total_loss = (actual_costs_tensor * log_probs.sum(dim=1)).sum() - (predicted_costs.detach() * log_probs.sum(dim=1)).sum() + (predicted_costs).sum()

        total_loss = torch.mul(torch.tensor(actual_costs_tensor, device=device), log_probs.sum(dim=1)).sum() \
                     - torch.mul(predicted_costs.detach(), log_probs.sum(dim=1)).sum() \
                     + predicted_costs.sum()

        grad_p = torch.autograd.grad(total_loss, cnn_model.parameters(), grad_outputs=torch.ones_like(total_loss), create_graph=True, retain_graph=True)
        grad_temp = torch.cat([torch.reshape(p, [-1]) for p in grad_p], 0)
        grad_ps = torch.square(grad_temp).mean(0)

        grad_s = torch.autograd.grad(grad_ps, surrogate_model.parameters(), grad_outputs=torch.ones_like(grad_ps), retain_graph=True, allow_unused=True)

        cnn_optimizer.zero_grad()
        total_loss.backward()

        cnn_optimizer.step()

        surrogate_optimizer.zero_grad()
        for params, grad in zip(surrogate_model.parameters(), grad_s):
            if grad is not None:
                params.grad = grad
        surrogate_optimizer.step()

        total_actual_cost += actual_costs_tensor.sum().item()
        total_epoch_loss += total_loss.item()
        total_batches_logged += 1

        print(f'epoch [{epoch + 1}/{num_epochs}], data index [{data_idx + 1}/10], total loss: {total_loss.item():.4f}')
        print(f'actual costs: {actual_costs_tensor.tolist()}')
        print(f'predicted costs: {predicted_costs.tolist()}')

        avg_actual_cost = total_actual_cost
        avg_epoch_loss = total_epoch_loss
        record.append(avg_actual_cost)
        loss_record.append(avg_epoch_loss)

        wandb.log({
            'epoch': epoch + 1,
            'data_index': data_idx + 1,
            'variance': grad_ps.item(),
            'cost_difference': cost_difference.mean().item(),
            'total_loss': total_loss.item(),
            'actual_costs': actual_costs_tensor.mean().item(),
            'avg_actual_cost': avg_actual_cost,
            'avg_epoch_loss': avg_epoch_loss,
            'predicted_costs': predicted_costs.mean().item(),
        })

        if (epoch + 1) % 10 == 0:
            validation_cost = validate_model(cnn_model, surrogate_model, validation_targets, validation_obstacles, validation_starts, device)
            wandb.log({'validation_actual_cost_every_10_epochs': validation_cost})

    print(f'total batches logged: {total_batches_logged}')
    wandb.finish()


if __name__ == '__main__':
    main()
