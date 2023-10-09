#!/home/icas/anaconda3/envs/morai/bin/python3
# Authors: Junior Costa de Jesus #

import rospy
import os
import json
import numpy as np
import random
import time
import sys
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
from collections import deque
from std_msgs.msg import Float32
from morai_env import Env
import torch
import torch.nn.functional as F
import torch.optim as optim
from torch.distributions import Normal
import gc
import torch.nn as nn
import math
from collections import deque
import copy
import pandas as pd

#---Directory Path---#
dirPath = os.path.dirname(os.path.realpath(__file__))

#****************************************************

class ReplayBuffer:
    def __init__(self, capacity):
        self.capacity = capacity
        self.buffer = []
        self.position = 0
    
    def push(self, state, action, reward, next_state, done):
        if len(self.buffer) < self.capacity:
            self.buffer.append(None)
        self.buffer[self.position] = (state, action, reward, next_state, done)
        self.position = (self.position + 1) % self.capacity
    
    def sample(self, batch_size):
        batch = random.sample(self.buffer, batch_size)
        state, action, reward, next_state, done = map(np.stack, zip(*batch))
        return state, action, reward, next_state, done
    
    def __len__(self):
        return len(self.buffer)


class ValueNetwork(nn.Module):
    def __init__(self, state_dim, hidden_dim, init_w=3e-3):
        super(ValueNetwork, self).__init__()
        
        self.linear1 = nn.Linear(state_dim, hidden_dim)
        self.linear2 = nn.Linear(hidden_dim, hidden_dim)
        self.linear2_3 = nn.Linear(hidden_dim, hidden_dim)
        self.linear3 = nn.Linear(hidden_dim, 1)
        
        self.linear1.weight.data.uniform_(-init_w, init_w)
        self.linear1.bias.data.uniform_(-init_w, init_w)
        self.linear2.weight.data.uniform_(-init_w, init_w)
        self.linear2.bias.data.uniform_(-init_w, init_w)
        self.linear2_3.weight.data.uniform_(-init_w, init_w)
        self.linear2_3.bias.data.uniform_(-init_w, init_w)
        self.linear3.weight.data.uniform_(-init_w, init_w)
        self.linear3.bias.data.uniform_(-init_w, init_w)
        
    def forward(self, state):
        x = mish(self.linear1(state))
        x = mish(self.linear2(x))
        x = mish(self.linear2_3(x))
        x = self.linear3(x)
        return x
        
        
class SoftQNetwork(nn.Module):
    def __init__(self, num_inputs, num_actions, hidden_size, init_w=3e-3):
        super(SoftQNetwork, self).__init__()
        
        self.linear1 = nn.Linear(num_inputs + num_actions, hidden_size)
        self.linear2 = nn.Linear(hidden_size, hidden_size)
        self.linear2_3 = nn.Linear(hidden_size, hidden_size)
        self.linear3 = nn.Linear(hidden_size, 1)
        
        self.linear1.weight.data.uniform_(-init_w, init_w)
        self.linear1.bias.data.uniform_(-init_w, init_w)
        self.linear2.weight.data.uniform_(-init_w, init_w)
        self.linear2.bias.data.uniform_(-init_w, init_w)
        self.linear2_3.weight.data.uniform_(-init_w, init_w)
        self.linear2_3.bias.data.uniform_(-init_w, init_w)
        self.linear3.weight.data.uniform_(-init_w, init_w)
        self.linear3.bias.data.uniform_(-init_w, init_w)
        
    def forward(self, state, action):
        x = torch.cat([state, action], 1)
        x = mish(self.linear1(x))
        x = mish(self.linear2(x))
        x = mish(self.linear2_3(x))
        x = self.linear3(x)
        return x
        
        
class PolicyNetwork(nn.Module):
    def __init__(self, num_inputs, num_actions, hidden_size, init_w=3e-3, log_std_min=-10, log_std_max=2):
        super(PolicyNetwork, self).__init__()
        
        self.log_std_min = log_std_min
        self.log_std_max = log_std_max
        
        self.linear1 = nn.Linear(num_inputs, hidden_size)
        self.linear1.weight.data.uniform_(-init_w, init_w)
        self.linear1.bias.data.uniform_(-init_w, init_w)
        
        self.linear2 = nn.Linear(hidden_size, hidden_size)
        self.linear2.weight.data.uniform_(-init_w, init_w)
        self.linear2.bias.data.uniform_(-init_w, init_w)
        
        self.mean_linear = nn.Linear(hidden_size, num_actions)
        self.mean_linear.weight.data.uniform_(-init_w, init_w)
        self.mean_linear.bias.data.uniform_(-init_w, init_w)
        
        self.log_std_linear = nn.Linear(hidden_size, num_actions)
        self.log_std_linear.weight.data.uniform_(-init_w, init_w)
        self.log_std_linear.bias.data.uniform_(-init_w, init_w)
        
    def forward(self, state):
        x = mish(self.linear1(state))
        x = mish(self.linear2(x))
        
        mean    = self.mean_linear(x)
        log_std = self.log_std_linear(x)
        log_std = torch.clamp(log_std, self.log_std_min, self.log_std_max)
        
        return mean, log_std
    
    def evaluate(self, state, epsilon=1e-6):
        mean, log_std = self.forward(state)
        std = log_std.exp()
        
        normal = Normal(mean, std)

        z = normal.sample()
        action = torch.tanh(z)
        
        log_prob = normal.log_prob(z) - torch.log(1 - action.pow(2) + epsilon)
        log_prob = log_prob.sum(-1, keepdim=True)
        
        return action, log_prob, z, mean, log_std
        
    
    def get_action(self, state, exploitation=False):
        state = torch.FloatTensor(state).unsqueeze(0)
        mean, log_std = self.forward(state)
        std = log_std.exp()
        
        normal = Normal(mean, std)
        z      = normal.sample()
        action = torch.tanh(z)
        if exploitation:
            action = torch.tanh(mean)
        #action = z.detach().numpy()
        
        action  = action.detach().numpy()
        return action[0]


def soft_q_update(batch_size, 
           gamma=0.99,
           mean_lambda=1e-3,
           std_lambda=1e-3,
           z_lambda=0.0,
           soft_tau=1e-2,
          ):
    state, action, reward, next_state, done = replay_buffer.sample(batch_size)

    state      = torch.FloatTensor(state)
    next_state = torch.FloatTensor(next_state)
    action     = torch.FloatTensor(action)
    reward     = torch.FloatTensor(reward).unsqueeze(1)
    done       = torch.FloatTensor(np.float32(done)).unsqueeze(1)
    #print('done', done)

    expected_q_value = soft_q_net(state, action)
    expected_value   = value_net(state)
    new_action, log_prob, z, mean, log_std = policy_net.evaluate(state)


    target_value = target_value_net(next_state)
    next_q_value = reward + (1 - done) * gamma * target_value
    q_value_loss = soft_q_criterion(expected_q_value, next_q_value.detach())

    expected_new_q_value = soft_q_net(state, new_action)
    next_value = expected_new_q_value - log_prob
    value_loss = value_criterion(expected_value, next_value.detach())

    log_prob_target = expected_new_q_value - expected_value
    policy_loss = (log_prob * (log_prob - log_prob_target).detach()).mean()
    

    mean_loss = mean_lambda * mean.pow(2).mean()
    std_loss  = std_lambda  * log_std.pow(2).mean()
    z_loss    = z_lambda    * z.pow(2).sum(1).mean()

    policy_loss += mean_loss + std_loss + z_loss

    soft_q_optimizer.zero_grad()
    q_value_loss.backward()
    soft_q_optimizer.step()

    value_optimizer.zero_grad()
    value_loss.backward()
    value_optimizer.step()

    policy_optimizer.zero_grad()
    policy_loss.backward()
    policy_optimizer.step()
    
    return q_value_loss.detach().numpy().tolist(), value_loss.detach().numpy().tolist(), policy_loss.detach().numpy().tolist()
    
    
    for target_param, param in zip(target_value_net.parameters(), value_net.parameters()):
        target_param.data.copy_(
            target_param.data * (1.0 - soft_tau) + param.data * soft_tau
        )


#---Mish Activation Function---#
def mish(x):
    '''
        Mish: A Self Regularized Non-Monotonic Neural Activation Function
        https://arxiv.org/abs/1908.08681v1
        implemented for PyTorch / FastAI by lessw2020
        https://github.com/lessw2020/mish
        param:
            x: output of a layer of a neural network
        return: mish activation function
    '''
    return torch.clamp(x*(torch.tanh(F.softplus(x))),max=6)

#----------------------------------------------------------

action_dim = 2
state_dim  = 16
hidden_dim = 500
ACTION_V_MIN = 0.0 # m/s
ACTION_W_MIN = -0.83 # rad/s
ACTION_V_MAX = 0.5 # m/s
ACTION_W_MAX = 0.83 # rad/s
world = 'morai'
model_folder = 'morai_RL_test'
value_net        = ValueNetwork(state_dim, hidden_dim)
target_value_net = ValueNetwork(state_dim, hidden_dim)

soft_q_net = SoftQNetwork(state_dim, action_dim, hidden_dim)
policy_net = PolicyNetwork(state_dim, action_dim, hidden_dim)

def hard_update(target,source):
    for target_param, param in zip(target.parameters(), source.parameters()):
        target_param.data.copy_(param.data)

value_criterion  = nn.MSELoss()
soft_q_criterion = nn.MSELoss()

value_lr  = 3e-4
soft_q_lr = 3e-4
policy_lr = 3e-4

value_optimizer  = optim.Adam(value_net.parameters(), lr=value_lr)
soft_q_optimizer = optim.Adam(soft_q_net.parameters(), lr=soft_q_lr)
policy_optimizer = optim.Adam(policy_net.parameters(), lr=policy_lr)

replay_buffer_size = 50000
replay_buffer = ReplayBuffer(replay_buffer_size)

print('State Dimensions: ' + str(state_dim))
print('Action Dimensions: ' + str(action_dim))
print('Action Max: ' + str(ACTION_V_MAX) + ' m/s and ' + str(ACTION_W_MAX) + ' rad/s')
#-----------------------------------------------------
def save_models(episode_count):
    if not os.path.exists(dirPath + '/SAC_Models/' + world + '/' + str(ensemble) + '/'):
                os.makedirs(dirPath + '/SAC_Models/' + world + '/' + str(ensemble) + '/')
    torch.save(policy_net.state_dict(), dirPath + '/SAC_Models/' + str(episode_count)+ '_policy_net.pth')
    torch.save(value_net.state_dict(), dirPath + '/SAC_Models/' + str(episode_count)+ '_value_net.pth')
    torch.save(soft_q_net.state_dict(), dirPath + '/SAC_Models/' + str(episode_count)+ '_soft_q_net.pth')
    torch.save(target_value_net.state_dict(), dirPath + '/SAC_Models/' + str(episode_count)+ '_target_value_net.pth')
    print("====================================")
    print("Model has been saved...")
    print("====================================")

def load_models(episode_count):
    policy_net.load_state_dict(torch.load(dirPath + '/SAC_Models/' + model_folder + '/' + str(episode_count)+ '_policy_net.pth'))
    value_net.load_state_dict(torch.load(dirPath + '/SAC_Models/' + model_folder + '/' + str(episode_count)+ '_value_net.pth'))
    soft_q_net.load_state_dict(torch.load(dirPath + '/SAC_Models/' + model_folder + '/' + str(episode_count)+ '_soft_q_net.pth'))
    target_value_net.load_state_dict(torch.load(dirPath + '/SAC_Models/' + model_folder + '/' + str(episode_count)+ '_target_value_net.pth'))
    print('***Models load***')


#****************************

SCAN_RANGE = 10
load_models(600)   

#----------------------------------------
def action_unnormalized(action, high, low):
    action = low + (action + 1.0) * 0.5 * (high - low)
    action = np.clip(action, low, high)
    return action
#**********************************


if __name__ == '__main__':
    rospy.init_node('sac_morai')
    print("SAC Start")
    env = Env()
    past_action = np.array([0.,0.])
    goal = False
    
    done = False
    if not goal:
        state = env.reset()
    goal = False
                
    while True:
        state = np.float32(state)
        print("state: ", state)
        action = policy_net.get_action(state, exploitation=True)
        # print("action: ", action)
        unnorm_action = np.array([action_unnormalized(action[0], ACTION_V_MAX, ACTION_V_MIN), action_unnormalized(action[1], ACTION_W_MAX, ACTION_W_MIN)])
        # unnorm_action[0] = 1.0
        next_state = env.step(unnorm_action, past_action)

        next_state = np.float32(next_state)

        state = copy.deepcopy(next_state)
        past_action = copy.deepcopy(action)


