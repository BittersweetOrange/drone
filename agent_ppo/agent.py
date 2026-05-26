#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
###########################################################################
# Copyright © 1998 - 2026 Tencent. All Rights Reserved.
###########################################################################
"""
Author: Tencent AI Arena Authors

Drone Obstacle Navigation Agent class based on kaiwudrl BaseAgent interface.
无人机避障导航 Agent 主类，基于 kaiwudrl BaseAgent 接口。
"""

import os
import torch
import numpy as np
from kaiwudrl.interface.agent import BaseAgent
from agent_ppo.model.model import ActorCritic
from agent_ppo.algorithm.algorithm import Algorithm
from agent_ppo.conf.conf import Config


class Agent(BaseAgent):
    def __init__(self, agent_type="player", device="cuda", logger=None, monitor=None):
        self.device = device
        self.logger = logger
        self.monitor = monitor
        self.obs_dim = Config.OBS_DIM
        self.action_dim = Config.ACTION_DIM

        self.model = ActorCritic(
            obs_dim=self.obs_dim,
            action_dim=self.action_dim,
            actor_hidden_dims=Config.ACTOR_HIDDEN_DIMS,
            critic_hidden_dims=Config.CRITIC_HIDDEN_DIMS,
            activation=Config.ACTIVATION,
            init_noise_std=Config.INIT_NOISE_STD,
            fixed_std=Config.FIXED_STD,
            use_layer_norm=Config.USE_LAYER_NORM,  # 【新增】传入 LayerNorm 配置
        ).to(device)

        self.algorithm = Algorithm(model=self.model, device=device, logger=logger, monitor=monitor)

    def _preprocess_obs(self, obs):
        """Unified observation preprocessing: type conversion + dimension flattening.
        统一的观测预处理：类型转换 + 维度展平。

        Args:
            obs: Raw observation, supports tuple/np.ndarray/torch.Tensor, shape [env_num, agent_num, obs_dim] or [batch_size, obs_dim].
            原始观测，支持 tuple/np.ndarray/torch.Tensor， 维度为 [env_num, agent_num, obs_dim] 或 [batch_size, obs_dim]。

        Returns:
            obs_flat: [batch_size, obs_dim] CUDA Tensor.
            original_shape: Original shape for restoring output dimensions.
        """
        if isinstance(obs, tuple):
            obs = obs[0]

        if isinstance(obs, np.ndarray):
            obs = torch.from_numpy(obs).float().to(self.device)
        elif obs.device != torch.device(self.device):
            obs = obs.to(self.device)

        original_shape = obs.shape
        if obs.shape[-1] != self.obs_dim:
            raise ValueError(f"Unexpected observation dim {obs.shape[-1]}, expected {self.obs_dim}")

        if obs.dim() == 3:
            obs = obs.view(obs.shape[0] * obs.shape[1], -1)

        return obs, original_shape

    def _reshape_output(self, original_shape, **tensors):
        """Restore output dimensions according to the original input shape.
        根据原始输入 shape 恢复输出维度。

        Args:
            original_shape: Original shape returned by _preprocess_obs.
            **tensors: Named tensors to reshape (e.g. actions, values, log_probs).

        Returns:
            Tuple of reshaped tensors in the same order as input.
        """
        if len(original_shape) == 3:
            env_num, agent_num = original_shape[0], original_shape[1]
            result = {}
            for name, t in tensors.items():
                if t.dim() == 1:
                    result[name] = t.view(env_num, agent_num)
                else:
                    result[name] = t.view(env_num, agent_num, -1)
            return tuple(result.values())
        return tuple(tensors.values())

    def predict(self, obs):
        """Predict actions in training mode (stochastic).
        训练模式下预测动作（随机采样）。

        Args:
            obs: [env_num, agent_num, obs_dim] or [batch_size, obs_dim].

        Returns:
            actions, values, log_probs.
        """
        obs, original_shape = self._preprocess_obs(obs)
        self.model.train()
        actions, values, log_probs = self.model(obs)
        return self._reshape_output(original_shape, actions=actions, values=values, log_probs=log_probs)

    def exploit(self, obs):
        """Exploit mode (deterministic actions).
        利用模式（确定性动作）。

        Args:
            obs: [env_num, agent_num, obs_dim] or [batch_size, obs_dim].

        Returns:
            actions.
        """
        obs, original_shape = self._preprocess_obs(obs)
        self.model.eval()
        with torch.no_grad():
            actions = self.model.act_inference(obs)
        (actions,) = self._reshape_output(original_shape, actions=actions)
        return actions

    def learn(self, training_data):
        """Run one learning update.
        执行一次学习更新。

        Args:
            training_data: Tuple of (obs, actions, old_log_probs, returns, advantages).

        Returns:
            Loss dictionary.
        """
        return self.algorithm.learn(training_data)

    def save_model(self, path=None, id="1"):
        """Save model parameters.
        保存模型参数。
        """
        model_file_path = f"{path}/model.ckpt-{str(id)}.pkl"
        torch.save(self.model.state_dict(), model_file_path)
        self.logger.info(f"saved model to {model_file_path}")

    def load_model(self, path=None, id="1"):
        """Load model parameters if the checkpoint exists.
        如果 checkpoint 存在则加载模型参数。
        """
        model_file_path = f"{path}/model.ckpt-{str(id)}.pkl"
        if os.path.exists(model_file_path):
            self.model.load_state_dict(torch.load(model_file_path, map_location=self.device))
            self.logger.info(f"loaded model from {model_file_path}")
        else:
            raise FileNotFoundError(f"model file not found: {model_file_path}")
