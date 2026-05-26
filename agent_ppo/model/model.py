#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
###########################################################################
# Copyright © 1998 - 2026 Tencent. All Rights Reserved.
###########################################################################
"""
Author: Tencent AI Arena Authors

Actor-Critic network for continuous control in Drone Obstacle Navigation.
无人机避障导航连续动作空间 Actor-Critic 网络。

Usage / 使用方式:
    model = ActorCritic(obs_dim, action_dim)
    actions, values, log_probs = model(observations)
"""

import torch

torch.set_num_interop_threads(1)
torch.set_num_threads(1)

import torch.nn as nn
import torch.nn.functional as F
from torch.distributions import Normal
from agent_ppo.conf.conf import Config


class FeatureExtractor(nn.Module):
    """
    特徵萃取網路：使用 Deep Sets (PointNet風格) 處理無序集合(障礙物、途徑點)。
    取代單純的 Flatten 拼接，讓網路具備實體感知能力。
    """
    def __init__(self, activation_fn, use_layer_norm=False):  # 【新增】use_layer_norm 参数
        super().__init__()
        # Agent global state: target(3)+linear_v(3)+angular_v(3)+rot(9)+hover(1)+start(3)+goal(3)+phase(2)+time(4) = 31 dims
        self.agent_dim = 31
        
        # 障礙物特徵處理 (Deep Sets): 位置(3) + 半徑(1) = 4 dims -> 32 dims
        # 【新增】在每层 Linear 后可选加入 LayerNorm，提升训练稳定性
        self.obs_mlp = nn.Sequential(
            nn.Linear(4, 32),
            nn.LayerNorm(32) if use_layer_norm else nn.Identity(),  # 【新增】
            activation_fn,
            nn.Linear(32, 64),
            nn.LayerNorm(64) if use_layer_norm else nn.Identity(),  # 【新增】
            activation_fn,
            nn.Linear(64, 32),
            activation_fn
        )
        
        # 途徑點特徵處理 (Deep Sets): 位置(3) + 狀態(1) = 4 dims -> 32 dims
        self.wp_mlp = nn.Sequential(
            nn.Linear(4, 32),
            nn.LayerNorm(32) if use_layer_norm else nn.Identity(),  # 【新增】
            activation_fn,
            nn.Linear(32, 64),
            nn.LayerNorm(64) if use_layer_norm else nn.Identity(),  # 【新增】
            activation_fn,
            nn.Linear(64, 32),
            activation_fn
        )

        self.output_dim = self.agent_dim + 32 + 32  # 31 + 32 + 32 = 95 dim

    def forward(self, obs):
        # 根據文檔解構 95 維空間
        # flatten shapes are [batch_size, 95]
        target_rpos = obs[:, 0:3]
        obstacle_rpos = obs[:, 3:27].view(-1, 8, 3)     # 8 個障礙物，每個 3 維
        linear_v = obs[:, 27:30]
        angular_v = obs[:, 30:33]
        rot_mat = obs[:, 33:42]
        hover_timer = obs[:, 42:43]
        obstacle_radii = obs[:, 43:51].view(-1, 8, 1)   # 8 個障礙物，每個 1 維
        start_rpos = obs[:, 51:54]
        goal_rpos = obs[:, 54:57]
        phase = obs[:, 57:59]
        waypoint_rpos = obs[:, 59:83].view(-1, 8, 3)    # 8 個 Waypoint，每個 3 維
        waypoint_vis = obs[:, 83:91].view(-1, 8, 1)     # 8 個 Waypoint，每個 1 維
        time_enc = obs[:, 91:95]

        # 1. 整理全局自身資訊 (31 dims)
        agent_state = torch.cat([
            target_rpos, linear_v, angular_v, rot_mat, hover_timer,
            start_rpos, goal_rpos, phase, time_enc
        ], dim=-1)

        # 2. 處理障礙物特徵
        obs_features = torch.cat([obstacle_rpos, obstacle_radii], dim=-1)  # [batch, 8, 4]
        obs_emb = self.obs_mlp(obs_features)                               # [batch, 8, 32]
        obs_pooled, _ = torch.max(obs_emb, dim=1)                          # [batch, 32]

        # 3. 處理途徑點特徵
        wp_features = torch.cat([waypoint_rpos, waypoint_vis], dim=-1)     # [batch, 8, 4]
        wp_emb = self.wp_mlp(wp_features)                                  # [batch, 8, 32]
        wp_pooled, _ = torch.max(wp_emb, dim=1)                            # [batch, 32]

        # 組合所有資訊成為新特徵 (95 dims)
        combined_features = torch.cat([agent_state, obs_pooled, wp_pooled], dim=-1)
        return combined_features


class ActorCritic(nn.Module):
    """Actor-Critic model with separate MLP networks and Gaussian action distribution.
    獨立 MLP 網路 + 高斯動作分佈的 Actor-Critic 模型。
    改進：加入 FeatureExtractor (Deep Sets) 解構環境特徵。
    """

    def __init__(
        self,
        obs_dim,
        action_dim,
        actor_hidden_dims=[256, 128, 64],
        critic_hidden_dims=[256, 128, 64],
        activation="elu",
        init_noise_std=1.0,
        fixed_std=False,
        use_layer_norm=False,  # 【新增】LayerNorm 开关
    ):
        super(ActorCritic, self).__init__()
        self.obs_dim = obs_dim
        self.action_dim = action_dim

        activation_fn = get_activation(activation)
        
        # 使用 Deep Sets 特徵萃取器
        self.feature_extractor = FeatureExtractor(activation_fn, use_layer_norm)  # 【修改】传入 use_layer_norm
        extracted_dim = self.feature_extractor.output_dim

        # Build the actor network.
        # 【新增】在隐藏层之间可选加入 LayerNorm
        actor_layers = []
        input_dim = extracted_dim
        for hidden_dim in actor_hidden_dims:
            actor_layers.append(nn.Linear(input_dim, hidden_dim))
            if use_layer_norm:  # 【新增】
                actor_layers.append(nn.LayerNorm(hidden_dim))
            actor_layers.append(activation_fn)
            input_dim = hidden_dim
        actor_layers.append(nn.Linear(input_dim, action_dim))
        self.actor = nn.Sequential(*actor_layers)

        # Build the critic network.
        # 【新增】在隐藏层之间可选加入 LayerNorm
        critic_layers = []
        input_dim = extracted_dim
        for hidden_dim in critic_hidden_dims:
            critic_layers.append(nn.Linear(input_dim, hidden_dim))
            if use_layer_norm:  # 【新增】
                critic_layers.append(nn.LayerNorm(hidden_dim))
            critic_layers.append(activation_fn)
            input_dim = hidden_dim
        critic_layers.append(nn.Linear(input_dim, 1))
        self.critic = nn.Sequential(*critic_layers)

        std = init_noise_std * torch.ones(action_dim)
        self.std = torch.tensor(std) if fixed_std else nn.Parameter(std)
        self.distribution = None

        Normal.set_default_validate_args = False
        self.init_weights()

    def init_weights(self):
        """Initialize weights with orthogonal initialization.
        使用正交初始化网络权重。
        """
        for module in self.modules():
            if isinstance(module, nn.Linear):
                torch.nn.init.orthogonal_(module.weight, gain=1.0)
                if module.bias is not None:
                    torch.nn.init.constant_(module.bias, 0.0)

    def forward(self, observations, history=None, masks=None, hidden_states=None):
        """Forward pass: sample actions and compute values.
        前向传播：采样动作并计算价值。

        Args:
            observations: [batch_size, obs_dim].

        Returns:
            actions: [batch_size, action_dim].
            values: [batch_size].
            log_probs: [batch_size].
        """
        if self.distribution is not None:
            del self.distribution
            self.distribution = None

        features = self.feature_extractor(observations)
        action_mean = self.actor(features)
        values = self.critic(features)

        # 【新增】限制标准差下界，防止 log_prob 计算出现 NaN
        std = torch.clamp(self.std.to(action_mean.device), min=1e-6)
        self.distribution = Normal(action_mean, std)

        actions = self.distribution.sample()
        log_probs = self.distribution.log_prob(actions).sum(dim=-1)

        return actions, values.squeeze(-1), log_probs

    @property
    def action_mean(self):
        return self.distribution.mean if self.distribution is not None else None

    @property
    def action_std(self):
        return self.distribution.stddev if self.distribution is not None else None

    @property
    def entropy(self):
        return self.distribution.entropy().sum(dim=-1) if self.distribution is not None else None

    def update_distribution(self, observations):
        """Update the action distribution given observations.
        根据观测更新动作分布。
        """
        features = self.feature_extractor(observations)
        mean = self.actor(features)
        # 【新增】限制标准差下界，防止 log_prob 计算出现 NaN
        std = torch.clamp(self.std.to(mean.device), min=1e-6)
        self.distribution = Normal(mean, std)

    def act(self, observations, deterministic=False):
        """Generate actions from observations.
        根据观测生成动作。

        Args:
            observations: [batch_size, obs_dim].
            deterministic: If True, return mean actions.

        Returns:
            actions: [batch_size, action_dim].
        """
        self.update_distribution(observations)

        if deterministic:
            return self.distribution.mean
        else:
            return self.distribution.sample()

    def get_actions_log_prob(self, actions):
        """Compute log probability of given actions.
        计算给定动作的对数概率。
        """
        if self.distribution is None:
            raise ValueError("Distribution not initialized. Call update_distribution first.")
        return self.distribution.log_prob(actions).sum(dim=-1)

    def evaluate(self, observations):
        """Evaluate the value of observations.
        评估观测的价值。

        Args:
            observations: [batch_size, obs_dim].

        Returns:
            values: [batch_size].
        """
        features = self.feature_extractor(observations)
        values = self.critic(features)
        return values.squeeze(-1)

    def act_inference(self, observations):
        """Deterministic action generation for inference/evaluation.
        推理/评估时的确定性动作生成。

        Args:
            observations: [batch_size, obs_dim].

        Returns:
            actions: [batch_size, action_dim].
        """
        with torch.no_grad():
            features = self.feature_extractor(observations)
            action_mean = self.actor(features)
        # 【新增】强制截断动作到合法范围 [-1, 1]，保障评测时不出现极端指令导致炸机
        return torch.clamp(action_mean, min=-1.0, max=1.0)

    def train(self, mode=True):
        super().train(mode)
        return self

    def test(self):
        return self.eval()


def get_activation(act_name):
    """Get activation function by name.
    根据名称获取激活函数。
    """
    if act_name == "elu":
        return nn.ELU()
    elif act_name == "selu":
        return nn.SELU()
    elif act_name == "relu":
        return nn.ReLU()
    elif act_name == "crelu":
        return nn.ReLU()
    elif act_name == "lrelu":
        return nn.LeakyReLU()
    elif act_name == "tanh":
        return nn.Tanh()
    elif act_name == "sigmoid":
        return nn.Sigmoid()
    else:
        print(f"Invalid activation function: {act_name}, using ReLU instead")
        return nn.ReLU()
