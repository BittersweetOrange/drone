#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
###########################################################################
# Copyright © 1998 - 2026 Tencent. All Rights Reserved.
###########################################################################
"""
Author: Tencent AI Arena Authors

PPO hyperparameters and model configuration for Drone Obstacle Navigation.
无人机避障导航 PPO 超参数及模型配置。
"""


class Config:
    # ========== Fixed Task Dimensions / 固定任务维度 ==========
    TASK_NAME = "ObstacleHover"
    OBS_DIM = 95
    ACTION_DIM = 4
    MAX_WAYPOINTS = 8
    OBSTACLE_FEATURE_DIM = 32
    TIME_ENCODING_DIM = 4

    # ========== Training Hyperparameters / 训练超参数 ==========
    LEARNING_RATE = 0.0005
    CLIP_PARAM = 0.1
    GAMMA = 0.995
    LAM = 0.95
    VALUE_LOSS_COEF = 0.5
    ENTROPY_COEF = 0.001
    MAX_GRAD_NORM = 10.0

    # ========== Training Schedule / 训练调度 ==========
    NUM_LEARNING_EPOCHS = 4
    NUM_MINI_BATCHES = 16
    NUM_STEPS_PER_ENV = 64

    # ========== Model Architecture / 模型结构 ==========
    ACTOR_HIDDEN_DIMS = [256, 128, 64]
    CRITIC_HIDDEN_DIMS = [256, 128, 64]
    ACTIVATION = "elu"
    INIT_NOISE_STD = 1.0
    FIXED_STD = False

    # 【新增】是否在网络隐藏层之间使用 LayerNorm
    USE_LAYER_NORM = True

    # ========== Reward Coefficients / 奖励系数 ==========
    REWARD_DISTANCE_SCALE = 0.6
    DISTANCE_REWARD_MULTIPLIER = 6.0
    GOAL_REACHED_REWARD = 100.0
    IN_ARENA_REWARD = 0.0
    OUT_OF_ARENA_PENALTY = 100.0

    # ========== Algorithm Improvements / 算法改进 ==========  # 【新增】整个区块
    # 学习率线性衰减配置
    LR_DECAY = True
    MIN_LR = 0.0                                          # 衰减到的最小学习率
    MAX_TRAINING_ITERATIONS = 10000                       # 预估的最大训练迭代次数，用于计算衰减比例

    # 熵系数衰减配置
    ENTROPY_COEF_DECAY = True
    MIN_ENTROPY_COEF = 0.0001                             # 衰减到的最小熵系数

    # 值函数裁剪配置
    USE_VALUE_CLIPPING = True
    VALUE_CLIP_PARAM = 0.2                                # 值函数裁剪范围，通常与策略裁剪一致或略大
