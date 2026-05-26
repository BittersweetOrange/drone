#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
###########################################################################
# Copyright © 1998 - 2026 Tencent. All Rights Reserved.
###########################################################################
import torch
from agent_ppo.conf.conf import Config
from isaac_env.reward_provider_base import RewardProviderBase


class RewardProcess(RewardProviderBase):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._history = {}
        self._step_cache = {}

    def bind_reward_state(self, reward_state):
        previous_num_envs = self._history.get("phase").shape[0] if self._history else None
        super().bind_reward_state(reward_state)
        current_num_envs = (
            int(reward_state.num_envs)
            if reward_state is not None and hasattr(reward_state, "num_envs")
            else None
        )
        force_reset = previous_num_envs is None or current_num_envs is None or previous_num_envs != current_num_envs
        self._ensure_history(force_reset=force_reset)
        self._step_cache = {}

    def _env(self):
        if self.reward_state is None:
            raise RuntimeError("RewardProcess requires a bound env before computing rewards.")
        return self.reward_state

    @staticmethod
    def _mask_to_float(mask: torch.Tensor) -> torch.Tensor:
        return mask.float().view(-1)

    def _ensure_history(self, force_reset: bool = False):
        env = self._env()
        num_envs = int(env.num_envs)
        device = env.device
        if force_reset or not self._history or self._history["goal_distance"].shape[0] != num_envs:
            self._history = {
                "goal_distance": torch.zeros(num_envs, device=device),
                "phase": torch.full((num_envs,), -1, dtype=torch.long, device=device),
                "episode_step": torch.full((num_envs,), -1, dtype=torch.long, device=device),
                "initialized": torch.zeros(num_envs, dtype=torch.bool, device=device),
                "waypoint_vis": torch.zeros(num_envs, 8, device=device),
                "nearest_wp_dist": torch.full((num_envs,), 100.0, device=device),
            }
            self._step_cache = {}

    def _drone_state(self):
        return self._env().drone_state

    def _drone_pos(self):
        return self._drone_state()[..., :3]

    def _current_step_id(self):
        return self._env().progress_buf.long().view(-1)

    def _prepare_step_context(self):
        env = self._env()
        self._ensure_history()
        step_id = self._current_step_id()
        current_phase = env.phase.long().view(-1)
        drone_pos = self._drone_pos()
        goal_position = env.goal_marker_positions
        goal_distance = torch.norm(goal_position - drone_pos, dim=-1).view(-1)

        history = self._history
        reset_mask = (~history["initialized"]) | (step_id <= 1) | (step_id < history["episode_step"])
        if reset_mask.any():
            history["goal_distance"][reset_mask] = goal_distance[reset_mask]
            history["phase"][reset_mask] = current_phase[reset_mask]
            if hasattr(env, "waypoint_visited"):
                history["waypoint_vis"][reset_mask] = env.waypoint_visited.float()[reset_mask]

        nearest_wp_dist = torch.full((env.num_envs,), 100.0, device=env.device)
        has_waypoints = torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)
        if hasattr(env, "waypoint_visited") and hasattr(env, "waypoint_marker_positions"):
            wp_vis = env.waypoint_visited.float()
            wp_pos = env.waypoint_marker_positions
            unvisited_mask = (1.0 - wp_vis)
            
            wp_exists = (wp_pos.abs().sum(dim=-1) > 1e-4)
            has_waypoints = wp_exists.any(dim=-1)
            
            dists = torch.norm(wp_pos - drone_pos.unsqueeze(1), dim=-1)
            dists = dists + (1.0 - unvisited_mask) * 1000.0
            nearest_wp_dist, _ = torch.min(dists, dim=-1)
            nearest_wp_dist = torch.where(has_waypoints, nearest_wp_dist, torch.full_like(nearest_wp_dist, 100.0))

        return {
            "goal_distance": goal_distance,
            "phase": current_phase,
            "step_id": step_id,
            "nearest_wp_dist": nearest_wp_dist,
            "has_waypoints": has_waypoints,
        }

    def _update_step_cache(self):
        env = self._env()
        ctx = self._prepare_step_context()
        history = self._history
        cached_step_id = self._step_cache.get("step_id")
        if cached_step_id is not None and torch.equal(cached_step_id, ctx["step_id"]):
            return self._step_cache

        reset_mask = (~history["initialized"]) | (ctx["step_id"] <= history["episode_step"])
        reset_mask = reset_mask.bool()
        nav_mask = ctx["phase"] == env.nav_phase
        hover_mask = ctx["phase"] == env.hover_phase

        raw_goal_progress = torch.clamp(
            history["goal_distance"] - ctx["goal_distance"],
            min=-env.arrival_radius,
            max=env.arrival_radius,
        )
        goal_progress = torch.where(reset_mask, torch.zeros_like(raw_goal_progress), raw_goal_progress)
        goal_progress = goal_progress * nav_mask.float()

        enter_hover = (history["phase"] == env.nav_phase) & (ctx["phase"] == env.hover_phase) & (~reset_mask)

        wp_progress = torch.zeros(env.num_envs, device=env.device)
        if hasattr(env, "waypoint_visited"):
            waypoint_vis = env.waypoint_visited.float()
            wp_progress = torch.clamp(waypoint_vis - history["waypoint_vis"], min=0.0).sum(dim=-1)
            wp_progress = torch.where(reset_mask, torch.zeros_like(wp_progress), wp_progress)
            history["waypoint_vis"].copy_(waypoint_vis)

        min_obs_dist = torch.full((env.num_envs,), 100.0, device=env.device)
        if hasattr(env, "obstacle_clearance"):
            min_obs_dist = env.obstacle_clearance
            if min_obs_dist.dim() > 1:
                min_obs_dist, _ = torch.min(min_obs_dist, dim=-1)
            min_obs_dist = min_obs_dist.view(-1)

        wp_dist_progress = torch.clamp(history["nearest_wp_dist"] - ctx["nearest_wp_dist"], min=0.0)
        wp_dist_progress = torch.where(reset_mask, torch.zeros_like(wp_dist_progress), wp_dist_progress)
        wp_dist_progress = wp_dist_progress * ctx["has_waypoints"].float()

        history["goal_distance"].copy_(ctx["goal_distance"])
        history["phase"].copy_(ctx["phase"])
        history["episode_step"].copy_(ctx["step_id"])
        history["nearest_wp_dist"].copy_(ctx["nearest_wp_dist"])
        history["initialized"].fill_(True)

        self._step_cache = {
            "step_id": ctx["step_id"].clone(),
            "goal_progress": goal_progress,
            "enter_hover": enter_hover.float(),
            "wp_progress": wp_progress,
            "wp_dist_progress": wp_dist_progress,
            "min_obs_dist": min_obs_dist,
            "nav_mask": nav_mask.float(),
            "hover_mask": hover_mask.float(),
            "goal_distance": ctx["goal_distance"],
            "has_waypoints": ctx["has_waypoints"],
        }
        return self._step_cache

    def _reward_target_progress(self, **kwargs):
        """终点引力：绝对主导，不可动摇"""
        env = self._env()
        scale = Config.REWARD_DISTANCE_SCALE
        multiplier = Config.DISTANCE_REWARD_MULTIPLIER
        target_progress = self._update_step_cache()["goal_progress"] / max(float(env.arrival_radius), 1.0e-6)
        return target_progress * scale * multiplier

    def _reward_goal_reached(self, **kwargs):
        reward_value = Config.GOAL_REACHED_REWARD
        return self._update_step_cache()["enter_hover"] * reward_value

    def _reward_in_arena(self, **kwargs):
        env = self._env()
        out_of_bounds = env._compute_out_of_bounds(self._drone_pos())
        in_arena_reward = Config.IN_ARENA_REWARD
        out_of_arena_penalty = Config.OUT_OF_ARENA_PENALTY
        return torch.where(
            out_of_bounds.view(-1),
            torch.full((env.num_envs,), -out_of_arena_penalty, device=env.device),
            torch.full((env.num_envs,), in_arena_reward, device=env.device),
        )

    # =====================================================================
    # 核心原则：导航阶段不限速！速度控制全部放在悬停阶段
    # =====================================================================

    def _reward_waypoint_collected(self, **kwargs):
        """途径点收集奖励 (大幅提高以鼓勵繞路獲取)"""
        reward_value = 100.0
        return self._update_step_cache()["wp_progress"] * reward_value

    def _reward_waypoint_guide(self, **kwargs):
        """途径点引导：強引導，權重必須大於終點引力，才能抵消繞遠路的懲罰"""
        cache = self._update_step_cache()
        # 終點引力對距離的懲罰系數約為 18.0，因此得分點引導設為 25.0 才有淨正收益
        return cache["wp_dist_progress"] * 25.0 * cache["nav_mask"]

    def _reward_obstacle_penalty(self, **kwargs):
        """预测性速度感知避障惩罚（原版全局限速器，不动！）"""
        env = self._env()
        cache = self._update_step_cache()
        min_dist = cache["min_obs_dist"].view(-1)
        vel = env.drone_state[..., 7:10]
        speed = torch.norm(vel, dim=-1).view(-1)

        safe_distance = 0.35
        warning_distance = 1.2

        static_penalty = torch.where(
            min_dist < safe_distance,
            (safe_distance - min_dist) * -5.0,
            torch.zeros_like(min_dist)
        )
        dynamic_penalty = torch.where(
            (min_dist < warning_distance) & (speed > 1.0),
            -1.0 * (speed / torch.clamp(min_dist, min=0.1)) * 0.2,
            torch.zeros_like(min_dist)
        )
        return static_penalty + dynamic_penalty

    def _reward_hover_stability(self, **kwargs):
        """【核心改进】悬停阶段才严控速度，导航阶段放任飞行
        参考代码的秘诀：
        1. +1.0 基础奖励 = 悬停存活正反馈，让网络想留在终点
        2. 温和距离惩罚 = 引导靠近中心
        3. 速度惩罚 = 在悬停阶段强力控速，让网络自己学会刹车
        关键改动：速度惩罚从0.5提高到1.5，只在悬停阶段生效
        """
        env = self._env()
        ctx = self._prepare_step_context()
        hover_mask = (ctx["phase"] == env.hover_phase).float().view(-1)

        # 距离惩罚：温和引导靠近中心
        dist_penalty = -ctx["goal_distance"].view(-1) * 2.0
        
        # 【关键】速度惩罚：悬停阶段加重！从原版0.5提到1.5
        # 这是唯一控制悬停过冲的手段，approach_braking已删除
        vel_penalty = -torch.norm(env.drone_state[..., 7:10], dim=-1).view(-1) * 1.5
        
        # 角速度惩罚
        ang_vel_penalty = -torch.norm(env.drone_state[..., 10:13], dim=-1).view(-1) * 0.2

        # 核心：悬停存活基础奖励
        base_reward = 1.0

        return hover_mask * (dist_penalty + vel_penalty + ang_vel_penalty + base_reward)

    def _reward_smoothness(self, **kwargs):
        """导航平滑度奖励"""
        env = self._env()
        cache = self._update_step_cache()
        nav_mask = cache["nav_mask"]
        ang_vel = env.drone_state[..., 10:13]
        return -torch.norm(ang_vel, dim=-1).view(-1) * 0.5 * nav_mask

    def _reward_approach_braking(self, **kwargs):
        """
        【新增修復】基於距離的自適應減速 (Approach Braking)
        遠處不限速保證得分效率，靠近 2m 內要求逐步減速，完美銜接懸停。
        """
        env = self._env()
        ctx = self._prepare_step_context()
        
        dist = ctx["goal_distance"].view(-1)
        speed = torch.norm(env.drone_state[..., 7:10], dim=-1).view(-1)
        
        # 只在導航階段生效
        nav_mask = (ctx["phase"] == env.nav_phase).float()
        # 啟動減速的範圍（距離終點 2.0 米以內）
        near_mask = (dist < 2.0).float()
        
        # 動態期望最大速度：距離越近，容許速度越慢 (最低容忍 0.5m/s)
        expected_max_speed = torch.clamp(dist * 1.5, min=0.5)
        
        # 計算超過容許速度的部分
        overspeed = torch.clamp(speed - expected_max_speed, min=0.0)
        
        # 在接近區超速，給予懲罰以逼迫提前煞車
        return -overspeed * 2.0 * nav_mask * near_mask


__all__ = ["RewardProcess"]
