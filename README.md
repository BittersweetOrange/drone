# drone
# 项目简介

## 任务目标

控制一架 Crazyflie 四旋翼无人机，从封闭空间一端的起点出发，自主避开随机生成的柱状障碍物群，途中收集路径上的途径点，到达另一端终点并稳定悬停。

任务由**导航阶段**和**悬停阶段**两段组成：

1. **导航阶段**：无人机从起点出发，避开障碍物、通过可选途径点，到达终点区域。
2. **悬停阶段**：进入终点区域后，连续悬停指定步数并保持位置精度。

## 环境介绍

基于 NVIDIA Isaac Sim（OmniDrones）构建的无人机三维飞行仿真环境。被控对象为与 Crazyflie 2.1 物理参数对齐的四旋翼无人机模型，采用 PIDrate（CTBR：Collective Thrust + Body Rate）控制模式。

| 参数     | 说明                                               |
| -------- | -------------------------------------------------- |
| 仿真平台 | NVIDIA Isaac Sim + OmniDrones                      |
| 机型     | 四旋翼（对齐 Crazyflie 物理参数）                  |
| 控制模式 | CTBR 角速率指令 → PIDRateController → 电机推力     |
| 动作空间 | 4 维连续 [-1, 1]（roll/pitch/yaw 角速度 + 推力比） |
| 观测空间 | 95 维 float32（全局观测）                          |
| 仿真步长 | dt = 0.02s                                         |

### 地图

封闭长方体空间（5m × 5m × 3m），四面墙壁 + 天花板 + 地面，无人机不得飞出边界。

- 起点：空间一端固定位置
- 终点：空间另一端固定位置
- 初始姿态：yaw = 0°（朝向 +x 方向，即终点方向），零速度悬停起飞

### 元素介绍

| 元素       | 说明                                                         |
| ---------- | ------------------------------------------------------------ |
| **无人机** | 四旋翼飞行器（对齐 Crazyflie 物理参数），质量约 27g，动作空间为 4 维连续 CTBR 指令（3 维角速度 + 1 维推力比） |
| **障碍物** | 从地面延伸至天花板的圆柱体，随机数量（2~5）和随机半径（离散 5 档：0.15~0.35m），位置随机生成 |
| **起点**   | 空间一端固定位置 [0.5, 2.5, 0.15]，无人机初始放置于此        |
| **终点**   | 空间另一端固定位置 [4.5, 2.5, 1.5]（虚拟点，无物理实体），到达判定半径 0.2m |
| **途径点** | 空间中虚拟点（无物理实体），可选加分项，随机数量（0~5），触达判定半径 0.2m |

在创建训练任务和评估任务时，上述元素的配置方式有所不同。具体请查看[开发指南-环境配置](https://tencentarena.com/docs/p-competition-drone_obstacle_nav/7.0.12/guidebook/dev-guide/env/#环境配置)部分。

## 计分规则

### 评分公式

每个任务的评分由**导航分**、**悬停分**和 **途径分**三个子项相加，满分 100 分。

```text
总分 = nav_score + hover_score + wp_score
```



### 子分计算

**nav_score（导航分）**

```text
nav_score = nav_coeff × w_nav × (0.8 × time_norm + 0.2 × smooth_norm) × 100
```



- `nav_coeff`：计分资格标记，到达终点区域为 1，否则为 0
- `w_nav = 0.25`：导航分权重
- `time_norm`：导航时间归一化子项，越快越高
- `smooth_norm`：飞行平滑度归一化子项，越平稳越高

**hover_score（悬停分）**

```text
hover_score = nav_coeff × hover_coeff × w_hover × clamp(1 - avg_position_error / 0.5, 0, 1) × 100
```



- `nav_coeff`：计分资格标记，到达终点区域为 1，否则为 0
- `hover_coeff`：悬停计分标记，成功完成悬停为 1，否则为 0
- `w_hover = 0.35`：悬停分权重
- `avg_position_error`：悬停期间的平均位置偏移

**wp_score（途径点分）**

```text
wp_score = nav_coeff × w_wp × (visited_count / total_waypoints) × 100
```



- `nav_coeff`：计分资格标记，到达终点区域为 1，否则为 0
- `w_wp = 0.40`：途径点分权重
- `visited_count`：已收集的 Waypoint 数
- `total_waypoints`：当局总 Waypoint 数

### 任务失败

以下情况 episode 立即终止，总分 = 0：

- 无人机飞出封闭空间边界
- 碰撞次数达到 max_collisions（测评锁定为 1）

## 轨迹回放工具

我们提供了**无人机轨迹 3D 回放工具**，用于可视化评估任务产出的飞行轨迹数据，帮助分析无人机的飞行路径、避障策略和悬停表现。

### 下载

| 平台    | 下载链接                                                     |
| ------- | ------------------------------------------------------------ |
| Windows | [replay_parser_win_v1.0.0.zip](https://kaiwu-prod-1252931805.cos.ap-shanghai.myqcloud.com/public/drone_replay_tools/replay_parser_win_v1.0.0.zip) |
| macOS   | [replay_parser_mac_v1.0.0.tar.gz](https://kaiwu-prod-1252931805.cos.ap-shanghai.myqcloud.com/public/drone_replay_tools/replay_parser_mac_v1.0.0.tar.gz) |

### 使用方法

1. **解压**：下载对应平台的压缩包并解压到任意目录。

2. **打开工具**：直接在浏览器中打开解压目录下的 `index.html` 文件。如果浏览器限制本地文件读取，可在解压目录下启动本地静态服务：

   - Windows：`py -m http.server 8765`
   - macOS：`python3 -m http.server 8765`

   然后在浏览器中访问 `http://localhost:8765`。

3. **导入轨迹**：点击页面顶部的「回放 JSON」按钮，选择评估任务产出的 `env_<id>.json` 轨迹文件。该文件可在平台评估详情中点击「下载日志」按钮获取，下载解压后位于 `result` 目录下。

4. **回放控制**：

   - **播放 / 暂停**：控制轨迹自动播放
   - **上一帧 / 下一帧**：逐帧查看
   - **时间轴**：拖动滑块跳转到任意时刻
   - **轨迹尾迹 / 速度箭头 / 标签**：勾选开关可切换显示

5. **视角操作**：

   - **平移**：在主画布上按住鼠标左键拖动
   - **旋转**：在右上角旋转方块上按住鼠标左键拖动
   - **缩放**：滚轮滚动
   - **重置视角**：点击「重置视角」按钮

# 环境详述

## 环境配置

在智能体和环境的交互中，首先会调用`env.reset`方法，该方法接受一个`usr_conf`参数，这个参数通过读取`train_env_conf.toml`文件的内容来实现定制化的环境配置。因此，用户可以通过修改`train_env_conf.toml`文件中的内容来调整环境配置。

```python
# usr_conf为用户传入的环境配置
env_obs = env.reset(usr_conf=usr_conf)
```



`train_env_conf.toml`中包含以下配置信息：

### 关键参数说明

| 配置项              | 类型    | 默认值                         | 取值范围               | 说明                                                       |
| ------------------- | ------- | ------------------------------ | ---------------------- | ---------------------------------------------------------- |
| num_envs            | int     | 2048                           | > 0                    | 并行环境数量（Isaac Sim GPU 并行仿真实例数）               |
| num_obstacles_range | int[2]  | [2, 5]                         | min ≥ 2, max ≤ 5       | 每局障碍物数量采样范围                                     |
| radius_choices      | float[] | [0.15, 0.20, 0.25, 0.30, 0.35] | 预定义离散值的非空子集 | 障碍物半径离散档位（米），同一 episode 内所有障碍物共用    |
| nav_max_steps       | int     | 1500                           | [100, 3000]            | 导航阶段最大步数，超过未到达终点判定超时（测评使用 1500）  |
| hover_max_steps     | int     | 150                            | [50, 500]              | 悬停阶段固定步数                                           |
| max_collisions      | int     | 5                              | [1, 5]                 | 碰撞上限，达到则 episode 终止（测评锁定为 1）              |
| num_waypoints_range | int[2]  | [0, 5]                         | min ≥ 0, max ≤ 5       | 每局 Waypoint 数量采样范围，设为 0 可退化为纯导航+悬停训练 |

### 奖励配置说明

奖励实现归属 `agent_ppo/feature/reward_process.py`，每个 `[rewards.<name>]` 段对应一个 `_reward_<name>()` 方法。`weight` 用于控制该项是否参与最终求和以及参与强度（设为 `0.0` 可禁用某项）。

| 奖励项          | 类别 | 说明                                                 |
| --------------- | ---- | ---------------------------------------------------- |
| target_progress | 导航 | 当前终点距离进展奖励：接近终点为正，远离为负         |
| goal_reached    | 导航 | 首次到达终点并切入悬停阶段的一次性奖励（默认 100.0） |
| in_arena        | 安全 | 场内存活奖励 / 出界惩罚（默认出界 -100.0）           |

> **💡 补充说明**：
>
> 1. **`train_env_conf.toml`文件中的配置仅在训练时生效**，请按上表数据描述进行配置。若配置错误，训练任务会变为"失败"状态，此时可以通过查看**env模块的错误日志**进行排查。
> 2. 若需调整模型评估任务时的配置，用户需要通过腾讯开悟平台创建评估任务并完成环境配置，详细参数见[智能体模型评估模式](https://tencentarena.com/docs/p-competition-drone_obstacle_nav/7.0.12/guidebook/dev-guide/agent/#模型评估模式)。
> 3. 代码包仅提供 3 项 baseline 奖励，用户可根据环境信息自行扩展更多奖励项，在 `reward_process.py` 中新增 `_reward_<name>()` 方法，并在 toml 中追加对应 `[rewards.<name>]` 段启用。

`train_env_conf.toml`采用的默认配置：

```yaml
[env_conf]
num_envs = 2048

[env_conf.task_obstaclehover]
num_obstacles_range = [2, 5]
radius_choices = [0.15, 0.20, 0.25, 0.30, 0.35]
nav_max_steps = 1500
hover_max_steps = 150
max_collisions = 5
num_waypoints_range = [0, 5]

# 奖励配置
[rewards.target_progress]
weight = 1.0

[rewards.goal_reached]
weight = 1.0

[rewards.in_arena]
weight = 1.0
```



## 环境信息

| 数据名      | 数据类型    | 数据描述                                       |
| ----------- | ----------- | ---------------------------------------------- |
| frame_no    | int         | 当前环境实例运行时的帧号                       |
| observation | Observation | 环境实例的观测信息（95维 float32 张量）        |
| terminated  | int         | 当前环境实例是否结束（出界/碰撞超限/悬停完成） |
| truncated   | int         | 当前环境实例是否异常或中断                     |
| extra_info  | ExtraInfo   | 环境实例的可选额外信息                         |

### 观测信息（observation）

观测为 95 维 float32 张量，shape `[num_envs, 1, 95]`，包含无人机自身状态、障碍物信息、目标点信息和时间编码：

| 字段名           | 索引    | 类型         | 说明                                            |
| ---------------- | ------- | ------------ | ----------------------------------------------- |
| target_rpos      | [0:3]   | float32[3]   | 终点相对位置（基于加噪位置计算）                |
| obstacle_rpos    | [3:27]  | float32[8×3] | 8 个障碍物相对位置展平，未激活槽位全 0          |
| linear_velocity  | [27:30] | float32[3]   | 机体线速度                                      |
| angular_velocity | [30:33] | float32[3]   | 机体角速度                                      |
| rotation_matrix  | [33:42] | float32[9]   | 旋转矩阵展平（3×3），取值 [-1, 1]               |
| hover_timer      | [42:43] | float32[1]   | 悬停阶段累计时长（秒），导航阶段为 0            |
| obstacle_radii   | [43:51] | float32[8]   | 8 个障碍物半径，未激活为 0                      |
| start_rpos       | [51:54] | float32[3]   | 起点相对位置                                    |
| goal_rpos        | [54:57] | float32[3]   | 终点相对位置                                    |
| phase_onehot     | [57:59] | float32[2]   | 导航/悬停阶段 one-hot（[1,0]=导航，[0,1]=悬停） |
| waypoint_rpos    | [59:83] | float32[8×3] | 8 个 Waypoint 相对位置展平                      |
| waypoint_visited | [83:91] | float32[8]   | 0=待访问，1=已访问或槽位未启用                  |
| time_encoding    | [91:95] | float32[4]   | 归一化进度编码                                  |

> **注意**：`target_rpos` 与 `goal_rpos` 当前实现相同，均为终点相对位置。若需要"当前最近途径点"信息，请在 agent 侧自行根据 `waypoint_rpos` 和 `waypoint_visited` 计算。

### 内部无人机状态（drone_state）

`drone_state` 是底层 Isaac/OmniDrones 从无人机实体读取的动力学状态快照，可以通过 `RewardStateSnapshot.drone_state` 读取它。

当前任务固定 `drone_model = Crazyflie`、`force_sensor = false`，因此 `drone_state` 的 shape 为 `[num_envs, 1, 32]`，类型为 `torch.Tensor` / `float32`。其中 `num_envs` 是并行环境数量，第二维为单智能体维度。坐标均为当前 env 的局部坐标系，单位为米、秒、弧度制相关单位。

| 字段                   | 索引      | shape     | 类型    | 典型范围                                               | 说明                                            |
| ---------------------- | --------- | --------- | ------- | ------------------------------------------------------ | ----------------------------------------------- |
| position               | `[0:3]`   | `[N,1,3]` | float32 | 有效飞行区约 x∈[0,5], y∈[0,5], z∈[0.1,3]；越界时可超出 | 无人机局部坐标 xyz，`_drone_pos()` 即取该字段   |
| quaternion             | `[3:7]`   | `[N,1,4]` | float32 | 每维 [-1,1]，整体近似单位四元数                        | 姿态四元数                                      |
| linear_velocity_world  | `[7:10]`  | `[N,1,3]` | float32 | 连续值，无硬裁剪                                       | 世界/环境局部坐标系线速度，单位 m/s             |
| angular_velocity_world | `[10:13]` | `[N,1,3]` | float32 | 连续值，无硬裁剪                                       | 世界/环境局部坐标系角速度，单位 rad/s           |
| linear_velocity_body   | `[13:16]` | `[N,1,3]` | float32 | 连续值，无硬裁剪                                       | 机体系线速度，由世界系速度旋转到机体系          |
| angular_velocity_body  | `[16:19]` | `[N,1,3]` | float32 | 连续值，无硬裁剪                                       | 机体系角速度                                    |
| heading                | `[19:22]` | `[N,1,3]` | float32 | 每维 [-1,1]                                            | 机体 x 轴在世界/环境局部坐标系中的方向向量      |
| lateral                | `[22:25]` | `[N,1,3]` | float32 | 每维 [-1,1]                                            | 机体 y 轴方向向量                               |
| up                     | `[25:28]` | `[N,1,3]` | float32 | 每维 [-1,1]                                            | 机体 z 轴方向向量                               |
| normalized_throttle    | `[28:32]` | `[N,1,4]` | float32 | [-1,1]                                                 | 4 个旋翼油门归一化值，来源为 `throttle * 2 - 1` |

### 奖励侧无人机快照（RewardStateSnapshot）

`RewardStateSnapshot` 是 baseline 奖励处理使用的只读快照，每个环境 step 都会从 live env 克隆一份，避免奖励代码直接改写仿真状态。用户在 `agent_ppo/feature/reward_process.py` 中通过 `self._env()` 取到的 `env` 就是该快照，而不是外层 `env.step()` 对象。

常用字段如下，`N` 表示 `num_envs`，`O=8` 表示固定障碍物槽位数，`W=8` 表示固定 Waypoint 槽位数：

| 字段                                                    | shape / 类型                     | 取值范围                                | 说明                                                       |
| ------------------------------------------------------- | -------------------------------- | --------------------------------------- | ---------------------------------------------------------- |
| device                                                  | torch.device                     | cuda/cpu                                | 快照张量所在设备                                           |
| num_envs                                                | int                              | > 0                                     | 并行环境数量                                               |
| nav_phase / hover_phase                                 | int                              | 0 / 1                                   | 导航、悬停阶段常量                                         |
| arrival_radius                                          | float                            | 默认 0.2                                | 到达终点区域半径，单位 m                                   |
| stall_progress_threshold                                | float                            | 默认 0.002                              | 进展过小判定阈值，供奖励塑形使用                           |
| nav_max_steps / hover_max_steps                         | int                              | 默认 1500 / 150                         | 导航最大步数、悬停固定步数                                 |
| drone_radius                                            | float                            | 默认 0.05                               | 碰撞检测使用的无人机半径，单位 m                           |
| obstacle_safe_distance                                  | float                            | 默认 0.3                                | 障碍物安全距离阈值，单位 m                                 |
| waypoint_collect_radius                                 | float                            | 默认等于 `arrival_radius`（当前为 0.2） | Waypoint 触达半径，单位 m                                  |
| max_collisions                                          | int                              | 训练默认 5，评估锁定 1                  | 碰撞上限                                                   |
| score_weight_nav / score_weight_hover / score_weight_wp | float                            | 默认 0.25 / 0.35 / 0.40                 | 导航、悬停、Waypoint 比赛分权重                            |
| nav_time_weight / nav_smooth_weight                     | float                            | 默认 0.8 / 0.2，≥0，二者和 > 0          | 导航时间子项和平滑度子项权重                               |
| phase                                                   | torch.long `[N]`                 | {0,1,2}                                 | 当前阶段：0=导航，1=悬停，2=结束                           |
| progress_buf                                            | torch.float32 `[N]`              | [0, max_episode_length]                 | 当前 episode 已运行步数（IsaacEnv 内部计数张量为 float32） |
| nav_steps / hover_steps                                 | torch.long `[N]`                 | ≥ 0                                     | 导航/悬停阶段累计步数                                      |
| arrival_step                                            | torch.long `[N]`                 | -1 或 ≥0                                | 首次进入终点区域的步数，未到达为 -1                        |
| hover_timer                                             | torch.float32 `[N]`              | ≥ 0                                     | 悬停累计时长，单位 s                                       |
| drone_state                                             | torch.float32 `[N,1,32]`         | 见上表                                  | 无人机完整动力学状态                                       |
| goal_marker_positions                                   | torch.float32 `[N,1,3]`          | 场地坐标范围内                          | 终点位置                                                   |
| obstacle_positions                                      | torch.float32 `[N,O,3]`          | 场地坐标或隐藏位置                      | 障碍物中心位置，未激活槽位会被移到隐藏位置                 |
| obstacle_radii                                          | torch.float32 `[N,O]`            | 0 或配置半径档位                        | 障碍物半径，未激活为 0                                     |
| active_obstacle_mask                                    | torch.bool `[N,O]`               | {False, True}                           | 障碍物槽位是否启用                                         |
| active_radius_bucket                                    | torch.long `[N,O]`               | 半径 bucket 索引                        | 障碍物半径模板槽位索引                                     |
| num_obstacles                                           | torch.long `[N]`                 | 配置采样范围                            | 当前 episode 障碍物数量                                    |
| base_obstacle_radius                                    | torch.float32 `[N,1]`            | 配置半径档位                            | 当前 episode 采样到的基础障碍物半径                        |
| active_waypoint_mask                                    | torch.bool `[N,W]`               | {False, True}                           | Waypoint 槽位是否启用                                      |
| waypoint_positions                                      | torch.float32 `[N,W,3]`          | 场地坐标或隐藏位置                      | Waypoint 位置                                              |
| waypoint_visited                                        | torch.bool `[N,W]`               | {False, True}                           | Waypoint 是否已访问                                        |
| waypoint_visited_step                                   | torch.long `[N,W]`               | -1 或 ≥0                                | Waypoint 首次访问步数，未访问为 -1                         |
| num_waypoints                                           | torch.long `[N]`                 | 配置采样范围                            | 当前 episode Waypoint 数量                                 |
| collision_count                                         | torch.long `[N]`                 | ≥ 0                                     | 累计碰撞次数                                               |
| timeout_flag                                            | torch.bool `[N]`                 | {False, True}                           | 是否导航超时                                               |
| arrival_success_flag                                    | torch.bool `[N]`                 | {False, True}                           | 是否曾进入终点区域                                         |
| hover_success_flag                                      | torch.bool `[N]`                 | {False, True}                           | 是否完成悬停                                               |
| hover_failed_flag                                       | torch.bool `[N]`                 | {False, True}                           | 是否悬停阶段失败                                           |
| success_flag                                            | torch.bool `[N]`                 | {False, True}                           | 是否最终成功                                               |
| collision_exceeded                                      | torch.bool `[N]`                 | {False, True}                           | 是否达到碰撞上限                                           |
| prev_collision_flag                                     | torch.bool `[N]`                 | {False, True}                           | 上一步是否处于碰撞状态，用于边沿检测                       |
| waypoint_score_sum                                      | torch.float32 `[N,1]`            | ≥ 0                                     | Waypoint 累计得分                                          |
| prev_action / last_applied_action                       | torch.float32 `[N,1,4]`          | [-1,1]                                  | 上一帧动作 / 实际下发到控制器的动作                        |
| prev_action_diff                                        | torch.float32 `[N,1,4]`          | 连续值                                  | 上一步动作差分                                             |
| action_diff / action_diff_jerk                          | `Optional[torch.float32[N,1,4]]` | 连续值                                  | 动作一阶差分 / 二阶差分，首步前可能为空                    |
| effort                                                  | `Optional[torch.float32[N,1]]`   | 连续值                                  | 底层 `drone.apply_action()` 返回的电机输出诊断量           |
| prev_linear_velocity                                    | torch.float32 `[N,1,3]`          | 连续值                                  | 上一步线速度缓存，用于计算平滑度                           |
| prev_angular_velocity                                   | torch.float32 `[N,1,3]`          | 连续值                                  | 上一步角速度缓存，用于计算平滑度                           |
| action_error_order1                                     | torch.float32 `[N,1]`            | ≥ 0                                     | 当前实现中等价于状态平滑代价 `state_smoothness_cost`       |
| action_error_order2                                     | torch.float32 `[N,1]`            | ≥ 0                                     | 归一化角加速度代价                                         |
| state_smoothness_cost                                   | torch.float32 `[N,1]`            | ≥ 0                                     | 当前步状态平滑代价                                         |
| smoothness_accum                                        | torch.float32 `[N,1]`            | ≥ 0                                     | 平滑代价累计和                                             |
| reward_return                                           | torch.float32 `[N,1]`            | 连续值                                  | 环境奖励累计回报                                           |
| prev_distance_to_target                                 | torch.float32 `[N,1]`            | ≥ 0                                     | 上一步到终点距离                                           |
| hover_error_sum / hover_error_count                     | torch.float32 `[N,1]`            | ≥ 0                                     | 悬停误差累计和 / 计数                                      |
| hover_precision                                         | torch.float32 `[N,1]`            | [0,1]                                   | 悬停精度归一化值                                           |
| obstacle_clearance                                      | torch.float32 `[N]`              | 连续值，可为负                          | 最近障碍物净空，≤0 表示几何碰撞                            |
| obstacle_collision / out_of_bounds                      | torch.bool `[N]`                 | {False, True}                           | 当前是否碰撞 / 是否越界                                    |

### 得分信息

`score_info` 是在当前状态下执行动作 `action` 所获得的分数，分数的计算详见[计分规则](https://tencentarena.com/docs/p-competition-drone_obstacle_nav/7.0.12/guidebook/dev-guide/intro/#计分规则)。

> **注意**：得分用于衡量模型在环境中的表现，也作为衡量强化学习训练产出模型的评价指标，与强化学习里的奖励`reward` 要区别开。

### 额外信息（extra_info）

| 字段名         | 类型                  | 说明                                                         |
| -------------- | --------------------- | ------------------------------------------------------------ |
| result_code    | int                   | 0=正常；-1=reset异常；-2=step异常                            |
| result_message | str                   | 错误详情或 "OK"                                              |
| stats          | dict[str, np.ndarray] | 环境监控与评分字段（key 集合固定，每个 value shape 为 [N, 1]） |

`stats` 关键字段：

| 字段名             | 类型            | 说明                |
| ------------------ | --------------- | ------------------- |
| success            | np.ndarray[N,1] | 最终是否完成任务    |
| timeout            | np.ndarray[N,1] | 是否导航超时        |
| collision_exceeded | np.ndarray[N,1] | 是否碰撞超限        |
| arrival_success    | np.ndarray[N,1] | 是否进入过终点区域  |
| hover_success      | np.ndarray[N,1] | 是否完成悬停        |
| hover_failed       | np.ndarray[N,1] | 是否悬停阶段失败    |
| total_score        | np.ndarray[N,1] | 最终比赛总分        |
| nav_coeff          | np.ndarray[N,1] | 导航计分资格标记    |
| hover_coeff        | np.ndarray[N,1] | 悬停计分标记        |
| nav_score_raw      | np.ndarray[N,1] | 导航归一化子分      |
| hover_score_raw    | np.ndarray[N,1] | 悬停归一化子分      |
| wp_score_raw       | np.ndarray[N,1] | Waypoint 归一化子分 |
| time_norm          | np.ndarray[N,1] | 导航时间归一化子项  |
| smooth_norm        | np.ndarray[N,1] | 平滑度归一化子项    |
| waypoints_visited  | np.ndarray[N,1] | 已访问 Waypoint 数  |
| waypoints_total    | np.ndarray[N,1] | 总 Waypoint 数      |
| collision_count    | np.ndarray[N,1] | 累计碰撞次数        |
| hover_precision    | np.ndarray[N,1] | 最终悬停精度        |

## 动作空间

动作为 4 维连续向量。底层控制模式为 **PIDrate（CTBR：Collective Thrust + Body Rate）**：策略网络输出经 `tanh` 压缩到 `[-1, 1]` 后，送入 PID 速率控制器转换为电机推力指令。

| 维度      | 策略归一化范围 | 物理含义          | 底层映射                                |
| --------- | -------------- | ----------------- | --------------------------------------- |
| action[0] | [-1, 1]        | 目标 roll 角速度  | 线性映射到 [-180°/s, +180°/s]           |
| action[1] | [-1, 1]        | 目标 pitch 角速度 | 线性映射到 [-180°/s, +180°/s]           |
| action[2] | [-1, 1]        | 目标 yaw 角速度   | 线性映射到 [-180°/s, +180°/s]           |
| action[3] | [-1, 1]        | 目标推力比        | 重映射为 (x+1)/2 并 clamp 到 [0.0, 1.0] |

### 合法动作

本任务为连续动作空间，不存在离散合法动作过滤。所有 4 维动作值均有效，经 `tanh` 激活后天然约束在 `[-1, 1]` 范围内。

### 时间信息

**步**(step)和**帧**(frame)存在一定映射关系。

**步**是强化学习环境中的一个时间单位，表示智能体(agent)在环境中执行一个动作并接收反馈的过程。在每一步中，智能体选择一个动作，环境根据该动作更新状态，并返回新的状态、奖励和终止信号。

**帧**是场景的一个时间单位，表示场景的一个完整更新周期。在每一帧中，场景的所有元素都会根据当前的状态和输入进行更新。

本任务中每一步对应一帧，即 step : frame = 1 : 1。episode 实际最大总步数 = `nav_max_steps` + `hover_max_steps`（默认 1500 + 150 = 1650 步）。

------

## 环境监控信息

监控面板中包含了**任务总览**、**障碍数量维度组（obstacle_num）**、**障碍半径维度组（obstacle_radius）**、**Waypoint 数量维度组（waypoint_num）**、**Waypoint 半径维度组（waypoint_radius）**四个维度组，每个维度组包含多个指标，每个指标对应一个监控面板，详细说明如下。

**任务总览组（task_overview）**

| 面板中文名称 | 面板英文名称    | 指标名称                                               | 说明                             |
| ------------ | --------------- | ------------------------------------------------------ | -------------------------------- |
| 任务结果     | task_funnel     | arrival_rate / success / failed / timeout              | 到达率、成功率、失败率、超时率   |
| 总分拆解     | score_breakdown | total_score / nav_score / hover_score / waypoint_score | 总分及三个子分项                 |
| 导航子项     | nav_components  | time_norm / smooth_norm                                | 导航时间归一化分与平滑度归一化分 |
| 过程质量     | process_quality | collision_count / max_collisions / hover_precision     | 碰撞次数、碰撞上限、悬停精度     |

**障碍数量维度组（obstacle_num）**

| 面板中文名称 | 面板英文名称                    | 指标名称                | 说明                           |
| ------------ | ------------------------------- | ----------------------- | ------------------------------ |
| 到达成功率   | arrival_rate_by_obstacle_num    | arrival_rate_n{2..5}    | 按障碍物数量分组的到达成功率   |
| 平均完成步数 | completion_cost_by_obstacle_num | completion_cost_n{2..5} | 按障碍物数量分组的平均完成步数 |
| 碰撞次数     | collision_count_by_obstacle_num | collision_count_n{2..5} | 按障碍物数量分组的平均碰撞次数 |

**障碍半径维度组（obstacle_radius）**

| 面板中文名称 | 面板英文名称                       | 指标名称                    | 说明                           |
| ------------ | ---------------------------------- | --------------------------- | ------------------------------ |
| 到达成功率   | arrival_rate_by_obstacle_radius    | arrival_rate_r{015..035}    | 按障碍物半径分组的到达成功率   |
| 平均完成步数 | completion_cost_by_obstacle_radius | completion_cost_r{015..035} | 按障碍物半径分组的平均完成步数 |
| 碰撞次数     | collision_count_by_obstacle_radius | collision_count_r{015..035} | 按障碍物半径分组的平均碰撞次数 |

**途径点数量维度组（waypoint_count）**

| 面板中文名称   | 面板英文名称                        | 指标名称                  | 说明                               |
| -------------- | ----------------------------------- | ------------------------- | ---------------------------------- |
| 平均总分       | total_score_by_waypoint_count       | total_score_w{1..5}       | 按 Waypoint 数量分组的平均总分     |
| 平均完成步数   | completion_cost_by_waypoint_count   | completion_cost_w{1..5}   | 按 Waypoint 数量分组的平均完成步数 |
| 获得途径点数量 | waypoints_visited_by_waypoint_count | waypoints_visited_w{1..5} | 按 Waypoint 数量分组的平均收集数   |

# 智能体详述

我们在代码包中提供了智能体的简单实现，本文将对该部分内容进行讲解，包括观测处理和奖励处理等。

## 观测处理

环境返回的`observation`信息为 95 维的观测向量，可以在`_preprocess_obs`方法中对这些观测信息进行预处理：

```python
    def _preprocess_obs(self, obs):
        """Unified observation preprocessing: type conversion + dimension flattening.
        统一的观测预处理：类型转换 + 维度展平。

        Args:
            obs: Raw observation, supports tuple/np.ndarray/torch.Tensor,
                 shape [env_num, agent_num, obs_dim] or [batch_size, obs_dim].
                 原始观测，支持 tuple/np.ndarray/torch.Tensor，
                 维度为 [env_num, agent_num, obs_dim] 或 [batch_size, obs_dim]。

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
```



预处理后的信息参考以下数据描述：

| 数据名         | 维度                       | 说明                                                |
| -------------- | -------------------------- | --------------------------------------------------- |
| obs            | [num_envs × agent_num, 95] | 展平后的观测张量，直接送入策略网络                  |
| original_shape | torch.Size                 | 原始输入 shape，用于 `_reshape_output` 恢复输出维度 |

### 特征分块说明

代码包直接将 95 维原始观测送入网络，未做特征增强。各维度含义如下：

| 特征块           | 索引范围 | 维度 | 说明                       |
| ---------------- | -------- | ---- | -------------------------- |
| target_rpos      | 0:3      | 3    | 终点相对位置               |
| obstacle_rpos    | 3:27     | 24   | 8 个障碍物相对位置展平     |
| linear_velocity  | 27:30    | 3    | 线速度                     |
| angular_velocity | 30:33    | 3    | 角速度                     |
| rotation_matrix  | 33:42    | 9    | 机体姿态旋转矩阵           |
| hover_timer      | 42:43    | 1    | 悬停累计时长               |
| obstacle_radii   | 43:51    | 8    | 障碍物半径                 |
| start_rpos       | 51:54    | 3    | 起点相对位置               |
| goal_rpos        | 54:57    | 3    | 终点相对位置               |
| phase_onehot     | 57:59    | 2    | 导航/悬停阶段 one-hot      |
| waypoint_rpos    | 59:83    | 24   | 8 个 Waypoint 相对位置展平 |
| waypoint_visited | 83:91    | 8    | 0=待访问, 1=已访问或未启用 |
| time_encoding    | 91:95    | 4    | 归一化时间编码             |

> **提示**：代码包仅使用原始观测作为 baseline。用户可自行实现特征工程（如追加"最近未访问 Waypoint 相对位置"、速度方向编码、障碍物方位分桶等），提升策略对导航目标的感知能力。追加特征时记得同步更新 `Config.OBS_DIM`。

### 奖励处理

代码包在 `agent_ppo/feature/reward_process.py` 中实现了 `RewardProcess` 类，作为 baseline 仅实现了 3 项核心奖励：

```python
class RewardProcess(RewardProviderBase):
    # 3 项 baseline 奖励函数
    def _reward_target_progress(self, **kwargs): ...     # 导航稠密进展
    def _reward_goal_reached(self, **kwargs): ...        # 到达终点稀疏奖励
    def _reward_in_arena(self, **kwargs): ...            # 出界惩罚
```



各奖励项的设计说明：

| 奖励项          | 触发条件         | 计算逻辑                                                     |
| --------------- | ---------------- | ------------------------------------------------------------ |
| target_progress | 导航阶段每步     | 对终点距离进展做归一化缩放（`REWARD_DISTANCE_SCALE=0.6`，`DISTANCE_REWARD_MULTIPLIER=6.0`） |
| goal_reached    | 首次进入终点区域 | 给予固定稀疏奖励（默认 100.0）                               |
| in_arena        | 每步             | 出界大惩罚（默认 -100.0），场内为 0                          |

奖励处理采用"只读 reward-state 快照 + 本地历史重建事件"的架构：

- 不直接访问 live env，而是每步绑定只读快照（`bind_reward_state`）
- 通过自维护历史缓存重建需要的瞬时事件（如 enter_hover、goal_progress 等）
- 同一 env step 内多个奖励项共享同一批事件重建结果（`_step_cache` 机制），避免重复计算

代码包仅提供最基础的奖励塑形。用户可以仔细阅读[环境详述](https://tencentarena.com/docs/p-competition-drone_obstacle_nav/7.0.12/guidebook/dev-guide/env/)和[数据协议](https://tencentarena.com/docs/p-competition-drone_obstacle_nav/7.0.12/guidebook/dev-guide/protocol/)，根据自己对环境的理解，自行扩展更多奖励项（例如 Waypoint 收集奖励、悬停精度奖励、障碍物接近惩罚、碰撞惩罚、动作平滑度惩罚等），不断提升智能体的能力。

------

## 算法介绍

我们在代码包中提供了PPO算法，同时还提供了一个**diy**模板算法文件夹，用户可在该文件夹中自定义算法实现。

| 算法名称 | 目录         | 说明                                      |
| -------- | ------------ | ----------------------------------------- |
| PPO      | `agent_ppo/` | 近端策略优化算法，连续动作空间 baseline   |
| DIY      | `agent_diy/` | 自定义算法模板，目录结构与 agent_ppo 一致 |

### 算法详细说明

**PPO（Proximal Policy Optimization）**

PPO 实现位于 `agent_ppo/algorithm/algorithm.py`，核心特性：

- **Actor-Critic 架构**：独立的 MLP 策略网络和价值网络
  - Actor：256 → 128 → 64 → 4（action_dim），ELU 激活
  - Critic：256 → 128 → 64 → 1，ELU 激活
- **高斯动作分布**：输出动作均值，标准差为可学习参数
- **GAE 优势估计**：γ = 0.995，λ = 0.95
- **PPO-Clip 更新**：clip_param = 0.1，每次采集 64 步 × num_envs 帧后进行 4 个 epoch × 16 个 mini-batch 的学习更新
- **梯度裁剪**：max_grad_norm = 10.0
- **正交初始化**：所有线性层权重采用正交初始化

关键超参数（定义在 `agent_ppo/conf/conf.py`）：

```python
class Config:
    OBS_DIM = 95             # 95 维原始观测
    ACTION_DIM = 4           # 4 维连续动作
    LEARNING_RATE = 0.0005
    CLIP_PARAM = 0.1
    GAMMA = 0.995
    LAM = 0.95
    VALUE_LOSS_COEF = 0.5
    ENTROPY_COEF = 0.001
    MAX_GRAD_NORM = 10.0
    NUM_LEARNING_EPOCHS = 4
    NUM_MINI_BATCHES = 16
    NUM_STEPS_PER_ENV = 64
    ACTOR_HIDDEN_DIMS = [256, 128, 64]
    CRITIC_HIDDEN_DIMS = [256, 128, 64]
    ACTIVATION = "elu"
```



## 算法监控信息

算法上报了reward等指标，用户可以通过腾讯开悟平台/客户端的监控功能查看。

针对当前算法的指标说明如下：

| 面板中文名称 | 面板英文名称 | 指标名称     | 说明                                   |
| ------------ | ------------ | ------------ | -------------------------------------- |
| 累积回报     | reward       | reward       | 每个 rollout batch 的平均累积奖励      |
| 总损失       | total_loss   | total_loss   | 策略损失 + 价值损失 × 0.5 - 熵 × 0.001 |
| 价值损失     | value_loss   | value_loss   | Critic 网络对 GAE 回报的 MSE 损失      |
| 策略损失     | policy_loss  | policy_loss  | PPO-Clip 裁剪后的策略梯度损失          |
| 熵损失       | entropy_loss | entropy_loss | 高斯策略分布的平均熵                   |

------

## 模型保存限制策略

为了避免用户保存模型的频率过于频繁，开悟平台对模型保存会有安全限制，不同的任务会有不同的限制，限制规则详情如下：

- 训练过程中每 **15 分钟** 自动保存一次模型 checkpoint
- 模型保存路径由平台自动管理，用户无需手动指定
- 保存操作由训练工作流中的 `agent.save_model()` 方法触发

------

## 模型评估模式

用户可以在腾讯开悟平台上创建模型评估任务。创建任务时，可以对该任务的环境进行配置，配置信息如下：

```yaml
# 评估任务环境配置
# Evaluation Environment Configuration
# 评估环境配置 - ObstacleHover 任务专用
[env_conf]
# Number of parallel environments (Isaac Sim GPU parallel simulation instances).
# Type: positive integer (range: min >= 1, max <= 50).
# Typical values still depend on available GPU/CPU/memory resources.
# 并行环境数量（Isaac Sim GPU 并行仿真实例数）。
# 类型：正整数（取值范围: min >= 1, max <= 50）。
# 实际可用取值仍取决于机器的 GPU/CPU/内存资源。
num_envs = 50

[env_conf.task_obstaclehover]
# Range of number of obstacles per episode [min, max].
# Each reset uniformly samples an integer from [min, max] as the obstacle count for that episode.
# Type: array of 2 integers, range: min >= 2, max <= 8.
# 每个 episode 的障碍物数量采样范围 [最小值, 最大值]。
# 每次 reset 时从 [min, max] 均匀采样一个整数作为本局障碍物数量。
# 类型：2 个整数的数组，取值范围：min >= 2, max <= 8。
num_obstacles_range = [2, 8]

# Obstacle radius discrete choices (meters).
# Each reset uniformly samples one radius from this list; all obstacles in the same episode share it.
# Only predefined discrete values are supported (bound to pre-created physics templates; continuous values not allowed).
# Available choices: [0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45]
# Type: array of floats (non-empty subset of available choices).
# 障碍物半径离散档位（米）。
# 每次 reset 时从列表中均匀采样一个半径，同一 episode 内所有障碍物共用该半径。
# 只能从以下预定义离散值中选取子集（与环境预创建的物理模板绑定，不支持连续值）。
# 可选值: [0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45]
# 类型：浮点数组（可选值的非空子集）。
radius_choices = [0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45]

# Maximum steps for navigation phase. Episode ends with timeout if goal is not reached within this limit.
# Simulation step dt = 0.02s, so 300 steps ≈ 6 seconds.
# Type: integer, range: [100, 3000], default: 1500.
# Note: evaluation uses 1500; training can use shorter values to accelerate feedback.
# 导航阶段最大步数，超过未到达终点则判定超时。
# 仿真步长 dt = 0.02s，300 步 ≈ 6 秒。
# 类型：整数，取值范围 [100, 3000]，默认 1500
nav_max_steps = 1500

# Fixed steps for hover phase. Duration for hover evaluation after entering the goal area.
# 150 steps = 3 seconds.
# Type: integer, range: [50, 500], default: 150.
# 悬停阶段固定步数，进入终点区域后的悬停评估时长。
# 150 步 = 3 秒。
# 类型：整数，取值范围 [50, 500]，默认 150。
hover_max_steps = 150

# Maximum allowed collisions before episode termination (task failure, score zeroed).
# Type: fixed integer 1 in evaluation (Crazyflie has no protection; collision = crash).
# 累计碰撞次数达到此值则终止 episode（任务失败，得分归零）。
# 类型：评估时固定为整数 1（Crazyflie 无保护，碰撞 = 炸机）。
# 注意：评估配置中该值不能修改。
max_collisions = 1

# Waypoint range per episode [min, max].
# waypoint 数量范围 [最小值, 最大值]。
num_waypoints_range = [3, 8]
```



> **注意**：测评时的障碍物参数范围严格大于训练集。`max_collisions` 在测评时锁定为 1（模拟真机碰撞即炸机），策略需具备更强的避障能力。

# 数据协议

为了方便同学们调用原始数据和特征数据，下面提供了协议供大家查阅。

## reset() 调用

```python
env_obs = env.reset(usr_conf=usr_conf)
```



返回结构：

```python
env_obs = {
    "env_id": str,              # 当前环境实例标识
    "frame_no": int,            # 帧号，从 reset 后第 1 帧开始
    "observation": np.ndarray,  # shape: [num_envs, 1, 95]
    "extra_info": {
        "result_code": int,     # 0=正常；-1=reset异常
        "result_message": str,
    },
    "terminated": np.ndarray | bool,
    "truncated": np.ndarray | bool,
}
```



## step() 调用

```python
env_reward, env_obs = env.step(actions)
```



返回结构：

```python
env_reward = {
    "env_id": str,
    "frame_no": int,
    "reward": np.ndarray,       # shape: [num_envs, 1, 1]，训练奖励（非比赛分数）
}

env_obs = {
    "env_id": str,
    "frame_no": int,
    "observation": np.ndarray,  # shape: [num_envs, 1, 95]
    "extra_info": {
        "result_code": int,
        "result_message": str,
        "stats": dict,          # 环境监控与评分字段
    },
    "terminated": np.ndarray,   # shape: [num_envs, 1]
    "truncated": np.ndarray,    # shape: [num_envs, 1]
}
```



## Observation 结构

观测为 95 维 float32 张量，shape `[num_envs, 1, 95]`：

| 字段名           | 索引    | 类型         | 取值范围      | 说明                                   |
| ---------------- | ------- | ------------ | ------------- | -------------------------------------- |
| target_rpos      | [0:3]   | float32[3]   | 连续          | 终点相对位置，基于加噪位置计算         |
| obstacle_rpos    | [3:27]  | float32[8×3] | 连续          | 8 个障碍物相对位置展平，未激活槽位全 0 |
| linear_velocity  | [27:30] | float32[3]   | 连续          | 机体线速度                             |
| angular_velocity | [30:33] | float32[3]   | 连续          | 机体角速度                             |
| rotation_matrix  | [33:42] | float32[9]   | [-1, 1]       | 旋转矩阵展平                           |
| hover_timer      | [42:43] | float32[1]   | ≥ 0           | 悬停阶段累计时长（秒）                 |
| obstacle_radii   | [43:51] | float32[8]   | 0 或半径档位  | 8 个障碍物半径，未激活为 0             |
| start_rpos       | [51:54] | float32[3]   | 连续          | 起点相对位置                           |
| goal_rpos        | [54:57] | float32[3]   | 连续          | 终点相对位置                           |
| phase_onehot     | [57:59] | float32[2]   | {[1,0],[0,1]} | 导航/悬停阶段 one-hot                  |
| waypoint_rpos    | [59:83] | float32[8×3] | 连续          | 8 个 Waypoint 相对位置展平             |
| waypoint_visited | [83:91] | float32[8]   | {0, 1}        | 0=待访问，1=已访问或槽位未启用         |
| time_encoding    | [91:95] | float32[4]   | [0, 1]        | 归一化进度编码                         |

## Action 结构

动作为 4 维连续向量。底层控制模式为 **PIDrate（CTBR）**：策略输出经 `tanh` 压到 `[-1, 1]`，再送入 PID 速率控制器：

| 维度      | 策略归一化范围 | 物理含义          | 底层映射                              |
| --------- | -------------- | ----------------- | ------------------------------------- |
| action[0] | [-1, 1]        | 目标 roll 角速度  | 线性映射到 [-180°/s, +180°/s]         |
| action[1] | [-1, 1]        | 目标 pitch 角速度 | 线性映射到 [-180°/s, +180°/s]         |
| action[2] | [-1, 1]        | 目标 yaw 角速度   | 线性映射到 [-180°/s, +180°/s]         |
| action[3] | [-1, 1]        | 目标推力比        | 重映射 (x+1)/2 并 clamp 到 [0.0, 1.0] |

## ExtraInfo 结构

| 字段名         | 类型                  | 说明                               |
| -------------- | --------------------- | ---------------------------------- |
| result_code    | int                   | 0=正常；-1=reset异常；-2=step异常  |
| result_message | str                   | 错误详情或 "OK"                    |
| stats          | dict[str, np.ndarray] | 环境监控与评分字段（key 集合固定） |

`stats` 关键字段：

| 字段名             | 类型            | 说明                |
| ------------------ | --------------- | ------------------- |
| success            | np.ndarray[N,1] | 最终是否完成任务    |
| timeout            | np.ndarray[N,1] | 是否导航超时        |
| collision_exceeded | np.ndarray[N,1] | 是否碰撞超限        |
| arrival_success    | np.ndarray[N,1] | 是否进入过终点区域  |
| hover_success      | np.ndarray[N,1] | 是否完成悬停        |
| hover_failed       | np.ndarray[N,1] | 是否悬停阶段失败    |
| total_score        | np.ndarray[N,1] | 最终比赛总分        |
| nav_coeff          | np.ndarray[N,1] | 计分资格标记        |
| hover_coeff        | np.ndarray[N,1] | 悬停计分标记        |
| nav_score_raw      | np.ndarray[N,1] | 导航归一化子分      |
| hover_score_raw    | np.ndarray[N,1] | 悬停归一化子分      |
| wp_score_raw       | np.ndarray[N,1] | Waypoint 归一化子分 |
| time_norm          | np.ndarray[N,1] | 导航时间归一化子项  |
| smooth_norm        | np.ndarray[N,1] | 平滑度归一化子项    |
| waypoints_visited  | np.ndarray[N,1] | 已访问 Waypoint 数  |
| waypoints_total    | np.ndarray[N,1] | 总 Waypoint 数      |
| collision_count    | np.ndarray[N,1] | 累计碰撞次数        |
| hover_precision    | np.ndarray[N,1] | 最终悬停精度        |

## 观测解析示例

```python
import numpy as np

def parse_env_obs(env_obs: dict) -> dict:
    obs = np.asarray(env_obs["observation"], dtype=np.float32)
    flat = obs[:, 0, :]
    result = {
        "target_rpos": flat[:, 0:3],
        "obstacle_rpos": flat[:, 3:27].reshape(-1, 8, 3),
        "linear_velocity": flat[:, 27:30],
        "angular_velocity": flat[:, 30:33],
        "rotation_matrix": flat[:, 33:42].reshape(-1, 3, 3),
        "hover_timer": flat[:, 42:43],
        "obstacle_radii": flat[:, 43:51],
        "start_rpos": flat[:, 51:54],
        "goal_rpos": flat[:, 54:57],
        "phase_onehot": flat[:, 57:59],
        "waypoint_rpos": flat[:, 59:83].reshape(-1, 8, 3),
        "waypoint_visited": flat[:, 83:91],
        "time_encoding": flat[:, 91:95],
        "terminated": np.asarray(env_obs["terminated"]).reshape(-1),
        "truncated": np.asarray(env_obs["truncated"]).reshape(-1),
    }
    return result
```



## 常见陷阱

1. `reward.reward` 不是比赛分数。比赛分数在 `extra_info.stats["total_score"]`，且通常只在 done 帧有最终值。
2. `target_rpos` 与 `goal_rpos` 当前实现相同，均为终点相对位置。若需要"当前最近途径点"信息，请在 agent 侧自行根据 `waypoint_rpos` 和 `waypoint_visited` 计算。
3. Waypoint 未激活槽位会被标为 `waypoint_visited = 1`，不能只看 visited 标记判断"本局已收集了多少 Waypoint"，需结合 `waypoint_rpos` 是否全 0 来判断槽位是否真正启用。
4. `terminated=True` 不一定表示成功，出界、碰撞超限、悬停失败也都走 `terminated=True`，需结合 `stats` 中的 `success` 字段判断任务结果。

## 错误码说明

| result_code | 含义         | 处理建议                               |
| ----------- | ------------ | -------------------------------------- |
| 0           | 正常         | 正常训练/评估                          |
| -1          | reset() 异常 | 检查 usr_conf、配置与 Isaac Sim 初始化 |
| -2          | step() 异常  | 检查动作 shape、GPU/Isaac 状态         |

---



## 训练流程简介

如图，完整训练流程包含以下关键环节：

| 环节                    | 介绍                                                         |
| ----------------------- | ------------------------------------------------------------ |
| **智能体-环境循环交互** | - 智能体将环境提供的观测和奖励处理为符合预测函数输入要求的数据； - 调用预测函数，生成动作指令； - 将智能体输出的动作指令处理为符合环境输入要求的数据； - 环境执行动作后完成状态转移，并反馈新的观测数据和奖励数据； |
| **样本处理**            | - 每个环境有不同的开始与结束逻辑，智能体与环境从开始到结束的完整交互过程，称为episode； - 智能体与环境每一次交互产生的结构化数据，称为**样本**；一个episode产生的样本序列称为**轨迹**； - 对轨迹数据进行处理，转换为规范化**训练样本(SampleData)**； |
| **模型迭代优化**        | - 基于训练样本，通过算法持续更新模型参数，实现策略优化；     |
| **智能体模型更新**      | - 智能体加载最新模型，与环境继续循环交互；                   |

该流程通过强化学习分布式计算框架提供的训练工作流实现。基于此，开发框架主要包含三大核心模块：

- [强化学习环境系统](https://tencentarena.com/docs/p-competition-drone_obstacle_nav/7.0.12/guidebook/taa-rl-fw/rl_env/)：提供标准的**强化学习环境接口**。开发者可以通过标准接口，实现智能体与环境的交互。
- [强化学习智能体开发套件](https://tencentarena.com/docs/p-competition-drone_obstacle_nav/7.0.12/guidebook/taa-rl-fw/rl_agent/info/)：提供标准的**强化学习智能体接口**，以及算法库、模型组件库等工具函数库。开发者可以通过工具函数库快速完成智能体的构建。
- [强化学习分布式计算框架](https://tencentarena.com/docs/p-competition-drone_obstacle_nav/7.0.12/guidebook/taa-rl-fw/distributed_computing_fw/)：提供标准接口，支持开发者按需实现训练工作流，运行单机或分布式的训练及评估任务。

## 代码包简介

开发者可以通过腾讯开悟平台所提供的强化学习项目使用开发框架。一个强化学习项目的代码目录如下：

```text
📦 根目录
├── 📂 agent
│   ├── 📂 algorithm
│       └── 📄 __init__.py
│       └── 📄 algorithm.py
│   ├── 📂 conf
│       └── 📄 __init__.py
│       └── 📄 conf.py
│       └── 📄 train_env_conf.toml
│   ├── 📂 feature
│       └── 📄 __init__.py
│       └── 📄 definition.py
│       └── 📄 preprocessor.py
│   ├── 📂 model
│       └── 📄 __init__.py
│       └── 📄 model.py
│   ├── 📂 workflow
│       └── 📄 __init__.py
│       └── 📄 train_workflow.py
│   ├── 📄 __init__.py
│   └── 📄 agent.py
├── 📂 conf
│   ├── 📄 __init__.py
│   ├── 📄 configure_app.toml
├── 📂 log
└── 📄 train_test.py
```



代码目录介绍：

| 目录名            | 介绍                                                         |
| ----------------- | ------------------------------------------------------------ |
| **agent/**        | 智能体子目录，智能体相关内容均集中于该目录，是开发者核心工作目录。 |
| **conf/**         | 配置文件目录，包含运行训练任务相关的配置，例如训练样本批处理大小batch_size等。 |
| **log/**          | 日志目录，存放运行代码测试脚本时生成的日志文件。             |
| **train_test.py** | 代码正确性测试脚本，该脚本会使用当前代码包完成一步训练。建议开发者在启动训练任务前，确保代码已通过该脚本检测。 |

### agent

| 目录/文件名    | 介绍                                                         |
| -------------- | ------------------------------------------------------------ |
| **algorithm/** | 算法相关，开发者在该目录下完成算法实现，包含loss计算、模型优化等，详情见[算法开发](https://tencentarena.com/docs/p-competition-drone_obstacle_nav/7.0.12/guidebook/taa-rl-fw/rl_agent/algorithm/) |
| **feature/**   | 特征相关，开发者在该目录下完成数据结构定义和数据处理方法，以及样本处理和奖励计算，详情见[数据处理与奖励设计](https://tencentarena.com/docs/p-competition-drone_obstacle_nav/7.0.12/guidebook/taa-rl-fw/rl_agent/feature/) |
| **model/**     | 模型相关，开发者在该目录下完成模型实现。 详情见[模型开发](https://tencentarena.com/docs/p-competition-drone_obstacle_nav/7.0.12/guidebook/taa-rl-fw/rl_agent/model/) |
| **workflow/**  | 工作流目录，开发者在该目录下完成训练工作流的开发。 详情见[工作流开发](https://tencentarena.com/docs/p-competition-drone_obstacle_nav/7.0.12/guidebook/taa-rl-fw/rl_agent/workflow/) |
| **agent.py**   | 智能体核心代码文件，开发者在该文件中完成预测、训练等核心函数的实现。 详情见[智能体开发](https://tencentarena.com/docs/p-competition-drone_obstacle_nav/7.0.12/guidebook/taa-rl-fw/rl_agent/agent/) |

标准代码包中都存在一个agent_diy子文件夹，该文件夹是预定义的智能体模板，可供开发者进行智能体的开发。

### conf

| 文件名                 | 介绍                                             |
| ---------------------- | ------------------------------------------------ |
| **configure_app.toml** | 训练任务相关的配置，包括样本大小、样本池大小等。 |

------

通过对训练流程和代码包的介绍，相信开发者能够对腾讯开悟开发框架建立了初步认知。

接下来，我们将详细介绍每个模块的功能及使用方式。

# 环境

在综述中提到，强化学习训练流程离不开智能体与环境的持续交互，本文将详细介绍强化学习环境系统的功能及标准接口函数。

## 概述

强化学习环境是基于输入动作，输出观测、奖励等反馈的功能模块，用于表达强化学习算法所求解的问题场景。

开发框架通过场景适配模块，对仿真器进行封装，将其特化的接口、协议转换为强化学习环境统一的接口和协议，供智能体调用。

强化学习环境系统主要提供如下功能：

1. 接收配置信息，用于指定自身初始化方式，比如环境中各种元素的初始状态。
2. 输出观测、奖励信息，可用于智能体预测、训练。
3. 输出观测、奖励之外的其他信息，供强化学习系统相关组件使用以实现特定功能。其他信息可包括可视化数据、日志数据等，实现的功能包括环境可视化、运行状况监测等。
4. 接收动作指令，完成状态转移并产生新的观测和奖励。

------

## 环境使用

开发框架通过场景适配模块，将问题场景进行标准化封装，为开发者提供统一的交互接口与通信协议。由于环境之间存在差异，接口中所涉及的观测、奖励等信息的具体数据结构也有所不同，开发者需查阅所使用环境的官方数据协议文档以获取准确信息。

开发者可以在[训练工作流](https://tencentarena.com/docs/p-competition-drone_obstacle_nav/7.0.12/guidebook/taa-rl-fw/distributed_computing_fw/#训练工作流)的workflow中获取到对应环境的实例，通过标准接口实现智能体与环境的交互。

**核心函数介绍**

### reset(usr_conf)

reset会将环境重置为环境配置文件中指定的状态，并且返回初始观测。

```python
# usr_conf为开发者传入的环境配置
obs, state = env.reset(usr_conf = usr_conf)
```



**Parameters**

| 参数名       | 介绍                   |
| ------------ | ---------------------- |
| **usr_conf** | dict类型，环境配置文件 |

**Returns**

| 参数名    | 介绍                   |
| --------- | ---------------------- |
| **obs**   | dict类型，环境观测信息 |
| **state** | dict类型，环境全局信息 |

------

### step(act, stop_game = false)

环境会执行传入的act动作指令，完成一次状态转移，并返回新的观测和奖励等信息。

```python
frame_no, _obs, score, terminated, truncated, _state = env.step(act, stop_game = false)
```



**Parameters**

| 参数名        | 介绍                       |
| ------------- | -------------------------- |
| **act**       | dict类型，环境执行的动作   |
| **stop_game** | bool类型，是否结束当前对局 |

**Returns**

| 参数名         | 介绍                                 |
| -------------- | ------------------------------------ |
| **frame_no**   | int类型，当前环境实例运行时的帧号    |
| **_obs**       | dict字典类型，当前帧的观测信息       |
| **score**      | int类型，当前帧的奖励信息            |
| **terminated** | bool类型，当前环境实例是否结束       |
| **truncated**  | bool类型，当前环境实例是否异常或中断 |
| **_state**     | dict字典类型，当前帧的全部状态信息   |

# 简介

智能体是强化学习系统中的核心模块，在[开发框架综述](https://tencentarena.com/docs/p-competition-drone_obstacle_nav/7.0.12/guidebook/taa-rl-fw/intro/)中提到，完整训练流程包括：

| 环节                    | 介绍                                                         |
| ----------------------- | ------------------------------------------------------------ |
| **智能体-环境循环交互** | - 智能体将环境提供的观测和奖励处理为符合预测函数输入要求的数据； - 调用预测函数，生成动作指令； - 将智能体输出的动作指令处理为符合环境输入要求的数据； - 环境执行动作后完成状态转移，并反馈新的观测数据和奖励数据； |
| **样本处理**            | - 每个环境有不同的开始与结束逻辑，智能体与环境从开始到结束的完整交互过程，称为episode； - 智能体与环境每一次交互产生的结构化数据，称为**样本**；一个episode产生的样本序列称为**轨迹**； - 对轨迹数据进行处理，转换为规范化**训练样本(SampleData)**； |
| **模型迭代优化**        | - 基于训练样本，通过算法持续更新模型参数，实现策略优化；     |
| **智能体模型更新**      | - 智能体加载最新模型，与环境继续循环交互；                   |

基于上述训练流程，我们将智能体的开发分为四个部分：

1. [数据处理及奖励设计](https://tencentarena.com/docs/p-competition-drone_obstacle_nav/7.0.12/guidebook/taa-rl-fw/rl_agent/feature/)：介绍基于环境观测数据进行特征处理、样本处理和奖励设计的方法。
2. [模型开发](https://tencentarena.com/docs/p-competition-drone_obstacle_nav/7.0.12/guidebook/taa-rl-fw/rl_agent/model/)：介绍模型开发接口及开发方法。
3. [算法开发](https://tencentarena.com/docs/p-competition-drone_obstacle_nav/7.0.12/guidebook/taa-rl-fw/rl_agent/algorithm/)：介绍包括算法开发接口及开发方法。
4. [工作流开发](https://tencentarena.com/docs/p-competition-drone_obstacle_nav/7.0.12/guidebook/taa-rl-fw/rl_agent/workflow/)：介绍开发者开发自定义训练工作流的方法。

接下来，将通过独立的章节对强化学习智能体开发套件中每个模块的功能及接口函数进行介绍。

# 数据处理与奖励设计

环境返回的数据通常无法直接作为智能体预测和训练的输入，开发者需要完成特征处理、样本处理和奖励设计，确保数据结构与类型符合智能体的接口规范。

## 特征处理

在特征处理时，开发者需要完成四个关键的开发工作，分别是**定义数据结构**、**观测处理**、**动作处理**。

### 定义数据结构

> 开发目录：`<智能体文件夹>/feature/definition.py`

首先，开发者需要定义智能体可以使用的数据结构（类）。

开发框架已经预先定义好了三种数据类型：ObsData, ActData, SampleData。

- ObsData和ActData分别表示智能体预测的输入和输出，将会由`agent.predict()`使用；
- SampleData为训练样本的数据类型，训练样本将会被`agent.learn()`使用，进行模型训练。

**核心函数介绍**

#### create_cls

用于动态创建数据结构（类）。ObsData, ActData, SampleData是训练流程必需的三类，但每一个类的数据结构包含哪些属性完全由开发者自定义，属性名称和属性数量没有限制。

```python
ObsData = create_cls("ObsData", 
    feature=None, 
)
ActData = create_cls("ActData",
    action=None,
    prob=None,
)
SampleData = create_cls("SampleData",
    npdata=None
)
```



**Parameters**

| 参数名         | 介绍                                     |
| -------------- | ---------------------------------------- |
| **第一个参数** | 字符串类型，类的名称                     |
| **其余参数**   | 类的属性，默认值为None，由开发者自行定义 |

------

### 观测处理

> 开发目录：`<智能体文件夹>/agent.py`

由于环境的reset和step接口返回的数据属于原始观测数据，无法直接作为智能体预测时的输入，开发者需要将这部分数据进行特征化。

**核心函数介绍**

#### observation_process

将环境返回的观测数据转换成ObsData类型数据。
很多情况下，特征工程包含了大量的数值处理、数据转换和领域知识，我们建议将大量的特征处理代码在`<智能体文件夹>/feature/preprocessor.py`文件中实现，然后由于`observation_process`进行调用。

```python
def observation_process(self, obs, state=None):
    return ObsData(feature=feature, legal_act=legal_actions)
```



**Parameters**

| 参数名    | 介绍                                                   |
| --------- | ------------------------------------------------------ |
| **obs**   | Observation类型，env.reset和env.step返回的环境观测数据 |
| **state** | EnvInfo类型，env.reset和env.step返回的环境状态数据     |

**Return**

| 参数名      | 介绍                                                         |
| ----------- | ------------------------------------------------------------ |
| **ObsData** | 开发者定义的ObsData类型的数据，将作为`agent.predict()`函数的输入。 |

------

### 动作处理

> 开发目录：`<智能体文件夹>/agent.py`

由于环境的step接口的输入须要满足环境的特定数据协议，开发者需要将智能体预测的输出转换为符合环境step接口输入要求的数据。

**核心函数介绍**

#### action_process

将智能体预测输出的ActData类型数据转换成环境可以接收的动作数据.

```python
def action_process(self, act_data):
    return act_data.act
```



**Parameters**

| 参数名       | 介绍                          |
| ------------ | ----------------------------- |
| **act_data** | 开发者定义的ActData类型的数据 |

**Return**

环境能处理的动作数据类型，作为`env.step()`的输入

------

## 奖励设计

> 开发目录：`<智能体名称>/feature/definition.py`

这里的奖励特指强化学习中的Reward，注意要与环境反馈的Score进行区分。Score通常用于衡量智能体在环境中的实际表现。开发者在设计Reward时，有非常大的灵活性，不仅可以基于环境返回的观测信息，还可以加入开发者对问题的理解、经验或者知识。

**核心函数介绍**

### reward_shaping

开发框架预设的奖励设计函数接口，开发者可以通过该函数实现复杂的奖励计算，在训练工作流中调用。

```python
def reward_shaping(obs, _obs, state, _state):
    return reward
```



**Parameters**

参数个数和类型不限制，可以是环境信息、智能体信息、开发者的经验和知识等。

**Return**

数值类型，计算出的reward值

------

## 样本处理

> 开发目录：`<智能体文件夹>/feature/definition.py`

由于环境与智能体交互过程中产生的轨迹数据无法直接作为智能体训练时的输入，开发者需要将轨迹数据转换为训练样本数据。

**核心函数介绍**

### sample_process

将环境与智能体交互过程中产生的轨迹数据转换成开发者定义的SampleData类型数据。

```python
@attached
def sample_process(self, list_game_data):
    return [SampleData(**i.__dict__) for i in list_game_data]
```



**Parameters**

| 参数名             | 介绍                                                         |
| ------------------ | ------------------------------------------------------------ |
| **list_game_data** | list(Frame)类型， 使用开发者自定义的Frame作为输入，因为样本一般进行批处理，所以传入列表 |

**Return**

| 参数名                   | 介绍                           |
| ------------------------ | ------------------------------ |
| **list(SampleData)类型** | SampleData类型的数据组成的列表 |

------

为了支持分布式训练，样本数据需要进行网络传输，由于SampleData无法直接进行网络传输，需要先转换成Numpy的Array，待传输到对端之后再由np.Array转换成SampleData。

因此，开发者需要实现两个转换函数 `SampleData2NumpyData`和 `NumpyData2SampleData`，这两个函数互为反函数。

> **注意**：由于这两个函数会被分布式计算框架调用，因此这两个函数的实现都必须包含一个装饰器@attached

### SampleData2NumpyData

将SampleData转换为NumpyData。

```python
@attached
def SampleData2NumpyData(g_data):
    return g_data.npdata
```



**Parameters**

| 参数名     | 介绍            |
| ---------- | --------------- |
| **g_data** | SampleData 类型 |

**Return**

Numpy.array类型

------

### NumpyData2SampleData

将NumpyData转换为SampleData。

```python
@attached
def NumpyData2SampleData(s_data):
    return SampleData(npdata=s_data)
```



**Parameters**

| 参数名     | 介绍             |
| ---------- | ---------------- |
| **s_data** | Numpy.array 类型 |

**Return type**

SampleData类型

# 模型开发

> 开发目录：`<智能体名称>/model/model.py`

一个强化学习模型是基于特征作为输入数据，输出策略的神经网络模型。

开发者需要在`model.py`文件中，实现神经网络模型。开发框架要求，模型类需继承 `torch.nn.Module` 类，即符合Pytorch模型的实现规范。

```python
class Model(nn.Module):
    def __init__(self, state_shape, action_shape=0, softmax=False):
        super().__init__()
```

# 算法开发

> 开发目录：`<智能体名称>/algorithm/algorithm.py`

在完成特征处理和奖励设计后，开发者还需要实现强化学习算法，以通过特定优化方法更新模型参数。

以下为实现强化学习算法的核心函数介绍，有关函数的更多细节可以查阅[分布式计算框架](https://tencentarena.com/docs/p-competition-drone_obstacle_nav/7.0.12/guidebook/taa-rl-fw/distributed_computing_fw/#agent)

**核心函数介绍**

### learn

实现强化学习优化算法的核心方法，该函数输入为训练样本数据，开发者需基于不同的算法完成相关实现，包括优化方法、损失计算等。

```python
def learn(self, list_sample_data):
    """
    Implementing the core method of the algorithm
    实现算法的核心方法
    """
    loss = 0                         # 基于不同算法实现loss计算 Calculate loss
    loss.backward()                  # 计算梯度 Calculate gradient
    self.optimizer.step()            # 通过梯度下降等方法更新模型 Update weights 
```



**Parameters**

| 参数名               | 介绍                               |
| -------------------- | ---------------------------------- |
| **list_sample_data** | list类型，训练样本(SampleData)列表 |

# 智能体开发

> 开发目录：`<智能体名称>/agent.py`

在完成模型和算法后，开发者还需要实现强化学习智能体，智能体使用模型进行决策、与环境交互并通过算法更新模型参数。

以下为实现强化学习智能体的核心函数介绍，有关函数的更多细节可以查阅[分布式计算框架](https://tencentarena.com/docs/p-competition-drone_obstacle_nav/7.0.12/guidebook/taa-rl-fw/distributed_computing_fw/#agent)

**核心函数介绍**

### learn

该函数输入为训练样本数据，开发者需要在该函数中调用算法消费训练样本进行训练。

当然，在不同的训练模式下，该函数使用方法有所不同：

- 单机训练：开发者需要在训练工作流中手动调用该函数以进行一步训练。
- 分布式训练：
  - 该函数作为训练函数会被循环执行，无需开发者手动调用。
  - 但该函数还作为样本发送函数，开发者需要在训练工作流中手动调用，以将样本发送至样本池。

```python
def learn(self, list_sample_data):
    self.algo.learn(list_sample_data)        # 调用算法消费训练样本进行训练 Call algorithm to train model
```



**Parameters**

| 参数名               | 介绍                               |
| -------------------- | ---------------------------------- |
| **list_sample_data** | list类型，训练样本(SampleData)列表 |

------

### predict

该方法通过调用模型进行预测，通常在训练时调用该方法，依策略的概率分布采样或引入随机概率。

```python
@predict_wrapper
def predict(self, list_obs_data, list_state):
    return [ActData]
```



**Parameters**

| 参数名            | 介绍                                       |
| ----------------- | ------------------------------------------ |
| **list_obs_data** | list类型，观测数据(ObsData)列表            |
| **list_state**    | 可选参数，list类型，环境返回的状态数据列表 |

**Return**

| 参数名            | 介绍                                        |
| ----------------- | ------------------------------------------- |
| **List(ActData)** | list类型，开发者定义的动作数据(ActData)列表 |

------

### exploit

该方法通过调用模型进行预测，通常在评估时调用该方法，选取策略中概率最高的动作或者策略认为最优的动作。

```python
@exploit_wrapper
def exploit(self, observation):
```



**Parameters**

| 参数名          | 介绍                                                         |
| --------------- | ------------------------------------------------------------ |
| **observation** | dict类型，环境观测字典，评估工作流中将原始的环境观测信息作为输入传入 `agent.exploit()`。 |

**Return**

| 参数名     | 介绍                                           |
| ---------- | ---------------------------------------------- |
| **action** | list类型，动作列表，环境可以直接使用的动作指令 |

------

### load_model

智能体通过该接口完成模型参数加载。在上文中提到，Actor会从模型池中获取最新模型参数文件，开发者需要手动调用`load_model()`函数，使智能体完成模型参数加载。

```python
@load_model_wrapper
def load_model(self, path=None, id="1"):
    # When loading the model, you can load multiple files,
    # and it is important to ensure that each filename matches the one used during the save_model process.
    # 加载模型, 可以加载多个文件, 注意每个文件名需要和save_model时保持一致
    model_file_path = f"{path}/model.ckpt-{str(id)}.pkl"
    self.model.load_state_dict(
        torch.load(model_file_path, map_location=self.device),
    )
```



**Parameters**

| 参数名   | 介绍                                                         |
| -------- | ------------------------------------------------------------ |
| **path** | string类型，加载模型参数文件的路径，开发框架根据使用场景得到相应的路径, 并作为输入传入 `load_model` |
| **id**   | string类型，模型参数文件的 id，使用 id 指定加载的模型参数文件 |

------

### save_model

开发者可以通过该函数保存当前时刻的模型文件及智能体代码包，开发框架会将开发者需要保存的内容打包为zip格式的文件。

当开发者使用腾讯开悟客户端开发时，开发框架会在客户端指定目录下存储该zip文件。
当开发者使用腾讯开悟平台时，开发框架会将该zip文件存储在云端，开发者可以通过平台的训练管理模块查看每一个训练任务的zip文件，即模型。

```python
@save_model_wrapper
def save_model(self, list_obs_data, list_state):
    # To save the model, it can consist of multiple files,
    # and it is important to ensure that each filename includes the "model.ckpt-id" field.
    # 保存模型, 可以是多个文件, 需要确保每个文件名里包括了model.ckpt-id字段
    model_file_path = f"{path}/model.ckpt-{str(id)}.pkl"

    # Copy the model's state dictionary to the CPU
    # 将模型的状态字典拷贝到CPU
    model_state_dict_cpu = {k: v.clone().cpu() for k, v in self.model.state_dict().items()}
    torch.save(model_state_dict_cpu, model_file_path)
```



**Parameters**

| 参数名   | 介绍                                                         |
| -------- | ------------------------------------------------------------ |
| **path** | string类型，模型文件保存的路径，开发框架根据使用场景得到相应的路径, 并作为输入传入 `save_model` |
| **id**   | string类型，模型文件的索引，开发框架获取到模型池中最新模型的索引, 并作为输入传入 `save_model` |

# 工作流开发

## 训练工作流

在完成智能体开发后，需要进一步实现由[分布式计算框架](https://tencentarena.com/docs/p-competition-drone_obstacle_nav/7.0.12/guidebook/taa-rl-fw/distributed_computing_fw/)提供的训练工作流接口，使智能体和环境持续交互，收集训练样本，迭代模型参数，最终完成策略的优化。

**核心函数介绍**

### workflow

通过该函数实现强化学习训练工作流，调用智能体和环境提供的接口，完成环境交互、样本收集和模型更新。

```python
@attached
def workflow(envs, agents, logger=None, monitor=None):
```



**Parameters**

| 参数名      | 介绍                                                         |
| ----------- | ------------------------------------------------------------ |
| **envs**    | list类型，环境列表，返回当前正在运行的环境。                 |
| **agents**  | list类型，智能体列表，通过调用开发者实现的 `<智能体名称>/agent.py` 实例化 Agent, 并作为输入传入 `workflow`。 |
| **logger**  | Logger类型，框架提供的日志组件，接口与 python 的 `logging` 库一致。 |
| **monitor** | Monitor类型，框架提供的监控组件。                            |

------

接下来，我们将通过一个训练工作流关键步骤的代码示例（具体实现由开发者完成），说明如何通过训练工作流实现完整训练流程。

```python
@attached
def workflow(envs, agents, logger=None, monitor=None):
    # Get the environment and agent
    # 获取环境和智能体
    env, agent = envs[0], agents[0]

    # Execute several epochs
    # 执行若干次epoch
    epoch_num = 1000
    
    # Each epoch executes several episodes
    # 每个epoch执行若干个episode
    episode_num_every_epoch = 1000

    # Training loop
    # 训练循环
    for epoch in range(epoch_num):
        # After each episode, the trajectory data is converted into training samples for training.
        # 在每一个episode结束之后，将轨迹数据转换成训练样本进行训练
        for g_data in run_episodes(episode_num_every_epoch, env, agent, logger, monitor):
            # Agent training. If single-machine training, the model is trained directly; if distributed training, samples are sent to the sample-pool.
            # agent进行训练。如果是单机训练，则直接对模型进行训练；如果是分布式训练，则将训练样本发送到样本池。
            agent.learn(g_data)
            # Ensure that the next training sample collected is new
            # 清空g_data，确保下一次搜集的训练样本是新的
            g_data.clear()
        
        # Save the model at intervals
        # 依据时间间隔保存模型
        now = time.time()
        if now - last_save_model_time >= 300:
            agent.save_model()
            last_save_model_time = now


def run_episodes(n_episode, env, agent, logger, monitor):
    # Run several episodes
    # 运行若干个episode
    for episode in range(n_episode):
        # Reset data at the beginning of an episode
        # 在episode开始时重置数据
        done = False
        collector = list()

        # Reset enviroment and get initial info
        # 重置环境, 并获取环境初始状态
        obs, state = env.reset(usr_conf=usr_conf)

        # Load the latest model and call it on demand; if in stand-alone mode, there is no need to load the remote model
        # 加载最新模型，按需调用；若训练采用单机模式，则无需加载远程模型，可不调用该函数
        agent.load_model(id="latest")

        # Run an episode loop
        # 运行一个episode循环
        while not done:
            # Agent performs inference, gets the predicted action for the next frame
            # 调用智能体预测函数，获取下一时刻的动作
            act_data = agent.predict(list_obs_data=[obs_data])[0]

            # Unpack ActData into action
            # 将智能体输出的ActData数据转换为符合环境数据协议要求的动作数据
            act = agent.action_process(act_data)

            # Interact with the environment, execute actions, get the next state
            # 调用环境step接口，与环境交互, 执行动作, 获取下一时刻的状态
            frame_no, _obs, score, terminated, truncated, _state = env.step(act)
            if _obs == None:
                break

            # Feature processing
            # 对环境返回的观测数据进行处理
            _obs_data = agent.observation_process(_obs, _state)

            # Disaster recovery
            # 容灾
            if truncated and frame_no == None:
                break

            # Calculate reward
            # 计算reward
            reward = reward_shaping(obs_data, _obs_data, state, _state)

            # Episode done signal
            # episode结束信号
            done = terminated or truncated

            # Construct sample
            # 构造样本
            frame = Frame(
                obs=obs_data.feature,
                _obs=_obs_data.feature,
                act=act,
                rew=reward,
                done=done,
            )
            collector.append(frame)

            # If the game is over, the sample is processed and sent to training
            # 如果episode结束，则进行样本处理，将样本送去训练
            if done:
                if len(collector) > 0:
                    collector = sample_process(collector)
                    # Return samples
                    # 返回样本数据, agent会调用agent.learn(g_data)进行训练
                    yield collector
                break

            # Status update
            # 状态更新
            obs_data = _obs_data
            obs = _obs
            state = _state
```

---



# 分布式计算框架

在强化学习项目的开发中，分布式计算框架是支撑大规模训练任务的核心基础设施。本开发框架提供了由腾讯王者荣耀团队自主研发的强化学习分布式计算框架KaiwuDRL，通过并行化计算、高效资源调度和分布式协同优化，显著提升智能体训练的效率与稳定性。

接下来，我们将详细介绍KaiwuDRL的系统架构与核心能力，帮助开发者进一步理解训练和评估工作流的运行逻辑。

## 总体架构

### 组件介绍

如上图所示，KaiwuDRL 的整体架构包括 Environment、Aisrv、Actor、Learner 等强化学习组件（均支持多实例并行运行）。此外，还集成了通信、日志、监控、对象存储等基础组件。

组件介绍如下表：

| 组件名称    | 功能描述                                                     |
| ----------- | ------------------------------------------------------------ |
| Environment | 环境服务组件，负责运行强化学习环境，支持通过标准接口与环境交互，并返回环境的观测obs。 |
| Aisrv       | 训练流程中枢，负责收集环境样本，运行训练、评估工作流，以及处理各个组件间的数据传输。 |
| Actor       | 预测服务组件，负责响应Aisrv的预测请求，调用智能体 predict() 或 exploit() 函数生成动作决策结果。 |
| Learner     | 训练服务组件，负责采集训练样本，调用智能体 learn() 函数完成梯度计算及模型迭代。 |
| MemoryPool  | 样本存储组件，简称样本池。负责存储训练样本，接收 Aisrv 打包的训练样本，发往 Learner 用于智能体训练。 |
| ModelPool   | 模型存储组件，简称模型池。负责存储模型参数文件，接收 Learner 产出的模型参数文件，将最新的模型参数文件发送给Actor。 |
| 日志        | 日志采集组件，负责记录强化学习系统中各个组件的运行日志，支持通过标准接口上报日志。 |
| 监控        | 监控采集组件，负责采集系统资源使用率、训练指标趋势等数据，支持通过标准接口上报数据指标。 |

> **MemoryPool和ModelPool仅在分布式训练时启用。**

### 服务介绍

基于上述组件，KaiwuDRL 提供了预测服务和训练服务：

#### 预测服务

1. Aisrv → Environment：发送环境配置并创建新一局episode；
2. Environment → Aisrv：返回原始观测数据；
3. Aisrv → Actor：Aisrv基于原始观测数据，向Actor发送预测请求；
4. Actor：使用预测请求中的观测进行特征处理，智能体基于特征处理后的数据进行预测，并且将预测数据处理为环境可以识别的动作指令，发送给Aisrv；
5. Aisrv：使用动作指令与环境Environment进行交互，Environment返回新的观测；

#### 训练服务

1. Aisrv：预测服务不断产生轨迹数据，Aisrv完成样本处理，并发送至样本池；
2. Learner：从样本池按批采集样本进行训练，并将最新的模型参数同步至Actor；

------

## 工作流

KaiwuDRL提供了训练、评估工作流的接口函数，开发者可以按需灵活调用上述组件和服务，以实现模型的训练和评估。

### 训练工作流

> 开发目录：`<智能体名称>/workflow/train_workflow.py`

#### workflow

训练工作流的核心函数，在workflow中可自定义训练流程。可以在[智能体/工作流开发](https://tencentarena.com/docs/p-competition-drone_obstacle_nav/7.0.12/guidebook/taa-rl-fw/rl_agent/workflow/)中查看详细的训练工作流代码示例。

```python
@attached
def workflow(envs, agents, logger=None, monitor=None):
```



**Parameters**

| 参数名      | 介绍                                                         |
| ----------- | ------------------------------------------------------------ |
| **envs**    | list类型，环境列表，返回当前正在运行的环境。                 |
| **agents**  | list类型，智能体列表，通过调用开发者实现的 `<智能体名称>/agent.py` 实例化 Agent, 并作为输入传入 `workflow`。 |
| **logger**  | Logger类型，框架提供的日志组件，接口与 python 的 `logging` 库一致。 |
| **monitor** | Monitor类型，框架提供的监控组件。                            |

------

### 评估工作流

在运行训练任务（训练工作流）并获得模型文件后，可以通过运行评估任务（评估工作流），对模型能力进行验证。

当开发者在使用腾讯开悟平台所提供的强化学习项目时，评估工作流由腾讯开悟官方实现，开发者无法修改。评估工作流会调用开发者自定义的`agent.exploit()`函数。

------

## Agent

KaiwuDRL提供了智能体相关的接口函数，开发者可以按需实现以下接口函数，并在训练工作流中调用。

> 开发目录：`<智能体名称>/agent.py`

### learn

该函数输入为训练样本数据，开发者需要在该函数中调用算法消费训练样本进行训练。

当然，在不同的训练模式下，该函数使用方法有所不同：

- 单机训练：开发者需要在训练工作流中手动调用该函数以进行一步训练。
- 分布式训练：
  - 该函数作为训练函数会被循环执行，无需开发者手动调用。
  - 但该函数还作为样本发送函数，开发者需要在训练工作流中手动调用，以将样本发送至样本池。

```python
@learn_wrapper
def learn(self, list_sample_data):
```



**Parameters**

| 参数名               | 介绍                                                         |
| -------------------- | ------------------------------------------------------------ |
| **list_sample_data** | list类型，训练样本(SampleData)列表，Learner从样本池按照配置项`train_batch_size`采样一批样本, 作为输入传入 `learn()` 函数。 |

**配置项**

> 开发目录: `/conf/configure_app.toml`

Learner在每一次执行`learn()`函数时，会从样本池采样一批样本作为输入。并按照开发者配置的频次`dump_model_freq`保存模型参数文件，模型同步服务将按照配置`model_file_sync_per_minutes`将模型参数文件推送至模型池。

Actor中的模型同步服务将按照配置`model_file_sync_per_minutes`，从模型池获取最新模型参数文件。

相关配置项如下：

```yaml
[app]
# The time interval for executing the learn() function, configurable to throttle the Learner and balance sample production/consumption.
# 执行learn函数进行训练的时间间隔，可通过该配置让Learner休息以调节样本生产消耗比
learner_train_sleep_seconds = 0.001

# Replay buffer configurations
# 样本池容量
replay_buffer_capacity = 4096

# The ratio of the sample pool capacity that triggers training
# 当样本池中的样本占总容量的比例达到该值时，启动训练
preload_ratio = 1.0

# When new samples are added to the sample pool, the logic for removing old samples: reverb.selectors.Lifo, reverb.selectors.Fifo
# 当新样本加入样本池时，旧样本的移除逻辑，可选项：reverb.selectors.Lifo, reverb.selectors.Fifo
# reverb.selectors.Lifo：先进后出(Last In, First Out)
# reverb.selectors.Fifo：先进先出(First In, First Out)
reverb_remover = "reverb.selectors.Fifo"

# The sampling logic of the Learner from the sample pool: reverb.selectors.Fifo, reverb.selectors.Uniform
# Learner从样本池中采样的逻辑，可选项：reverb.selectors.Fifo, reverb.selectors.Uniform
# reverb.selectors.Uniform：Samples are selected uniformly at random from the replay buffer, with each sample having an equal probability of being chosen.
# reverb.selectors.Uniform：从回放缓冲区中随机均匀地选择样本，每个样本被选中的概率相同。
# reverb.selectors.Fifo：Samples are selected in the order they were added to the replay buffer.
# reverb.selectors.Fifo：按照先进先出从回放缓冲区中选择样本。
reverb_sampler = "reverb.selectors.Uniform"

# Training batch size limit for Learner
# Learner训练时样本批处理大小
train_batch_size = 2048

# Model dump frequency (steps)
# 训练间隔多少步输出模型参数文件
dump_model_freq = 1000

# The Learner pushes model updates, and the frequency at which Actors fetch the model (in minutes).
# Learner推送模型参数文件至模型池，以及Actor从模型池获取模型参数文件的频次（单位：分钟）
model_file_sync_per_minutes = 1

# he number of model updates pushed per learner iteration, and the maximum number of updates each actor can fetch at once (cap: 50).
# Learner每次推送模型参数文件，以及Actor每次获取模型参数文件的数量（上限：50）
modelpool_max_save_model_count = 1
```



------

### predict

该方法通过调用模型进行预测，通常在训练时调用该方法，依策略的概率分布采样或引入随机概率。

```python
@predict_wrapper
def predict(self, list_obs_data, list_state):
    return [ActData]
```



**Parameters**

| 参数名            | 介绍                                       |
| ----------------- | ------------------------------------------ |
| **list_obs_data** | list类型，观测数据(ObsData)列表            |
| **list_state**    | 可选参数，list类型，环境返回的状态数据列表 |

**Return**

| 参数名            | 介绍                                        |
| ----------------- | ------------------------------------------- |
| **List(ActData)** | list类型，开发者定义的动作数据(ActData)列表 |

------

### exploit

该方法通过调用模型进行预测，通常在评估时调用该方法，选取策略中概率最高的动作或者策略认为最优的动作。

```python
@exploit_wrapper
def exploit(self, observation):
```



**Parameters**

| 参数名          | 介绍                                                         |
| --------------- | ------------------------------------------------------------ |
| **observation** | dict类型，环境观测字典，评估工作流中将原始的环境观测信息作为输入传入 `agent.exploit()`。 |

**Return**

| 参数名     | 介绍                                           |
| ---------- | ---------------------------------------------- |
| **action** | list类型，动作列表，环境可以直接使用的动作指令 |

------

### load_model

智能体通过该接口完成模型参数加载。在上文中提到，Actor会从模型池中获取最新模型参数文件，开发者需要手动调用`load_model()`函数，使智能体完成模型参数加载。

```python
@load_model_wrapper
def load_model(self, path=None, id="1"):
  # When loading the model, you can load multiple files,
    # and it is important to ensure that each filename matches the one used during the save_model process.
    # 加载模型, 可以加载多个文件, 注意每个文件名需要和save_model时保持一致
    model_file_path = f"{path}/model.ckpt-{str(id)}.pkl"
    self.model.load_state_dict(
        torch.load(model_file_path, map_location=self.device),
    )
```



**Parameters**

| 参数名   | 介绍                                                         |
| -------- | ------------------------------------------------------------ |
| **path** | string类型，加载模型参数文件的路径，开发框架根据使用场景得到相应的路径, 并作为输入传入 `load_model` |
| **id**   | string类型，模型参数文件的 id，使用 id 指定加载的模型参数文件 |

------

### save_model

开发者可以通过该函数保存当前时刻的模型文件及智能体代码包，开发框架会将开发者需要保存的内容打包为zip格式的文件。

当开发者使用腾讯开悟客户端开发时，开发框架会在客户端指定目录下存储该zip文件。
当开发者使用腾讯开悟平台时，开发框架会将该zip文件存储在云端，开发者可以通过平台的训练管理模块查看每一个训练任务的zip文件，即模型。

```python
@save_model_wrapper
def save_model(self, path=None, id="1"):
      # To save the model, it can consist of multiple files,
    # and it is important to ensure that each filename includes the "model.ckpt-id" field.
    # 保存模型, 可以是多个文件, 需要确保每个文件名里包括了model.ckpt-id字段
    model_file_path = f"{path}/model.ckpt-{str(id)}.pkl"

    # Copy the model's state dictionary to the CPU
    # 将模型的状态字典拷贝到CPU
    model_state_dict_cpu = {k: v.clone().cpu() for k, v in self.model.state_dict().items()}
    torch.save(model_state_dict_cpu, model_file_path)
```



**Parameters**

| 参数名   | 介绍                                                         |
| -------- | ------------------------------------------------------------ |
| **path** | string类型，模型文件保存的路径，开发框架根据使用场景得到相应的路径, 并作为输入传入 `save_model` |
| **id**   | string类型，模型文件的索引，开发框架获取到模型池中最新模型的索引, 并作为输入传入 `save_model` |

------

## 其他功能

### 加载预训练模型

本框架支持在已有的模型（称为预训练模型）基础上继续训练。预训练模型结构和继续训练的模型结构需保持一致。

- 当使用腾讯开悟平台时，在创建训练任务时，可以在弹窗中选择预训练模型以继续训练。
- 当使用腾讯开悟客户端时，需要将训练好的模型文件（`客户端工作空间/ckpt`路径下的pkl类型文件）放入代码包中，并在代码包`conf/configure_app.toml`中完成预训练模型相关的配置，配置说明如下：

```yaml
# Whether to enable the preload model function. If enabled (true), the model specified by preload_model_id will be loaded as the initial model in the preload_model_dir directory; if disabled (false), no preloading will be performed.
# 是否启用加载预训练模型功能，若开启(true)，将在preload_model_dir目录下加载由preload_model_id指定的模型作为初始模型；若关闭(false)，则不加载预训练模型。
preload_model = false

# The relative path of the preloaded model folder (the variable name {agent_name} refers to the agent_algorithm name directory in the code package). It is only effective when preload_model=true. When the preload model function is enabled, you need to create a new ckpt folder under the agent_algorithm name directory in the code package and place the model file (.pkl) there.
# 预训练模型文件夹相对路径(变量名{agent_name}指代码包中agent_算法名目录)，仅在preload_model=true时生效；当开启加载预训练模型功能时，需要在代码包中agent_算法名目录下新建ckpt文件夹，将模型文件（.pkl）放置此即可。
preload_model_dir = "{agent_name}/ckpt"

# The identification ID of the preloaded model (here refers to the number of model training steps). This ID corresponds to the number of training steps recorded in the model file name. It only takes effect when preload_model=true.
# Note that it is forbidden to modify the original model file name, otherwise the model preloading process will fail.
# 预训练模型的标识ID（这里指模型训练步数），该ID对应模型文件名中的训练步数记录。仅在preload_model=true时生效。
# 注意，禁止修改原始模型文件名，否则将导致预训练模型加载失败。
preload_model_id = 1000
```



> **客户端继续训练使用示例**
>
> 以加载 40052步的DQN模型 为例：
>
> - 解压模型压缩包，在ckpt路径下找到名为 `model.ckpt-40052.pkl` 的模型文件
> - 将模型文件放到代码包路径下，例如在`agent_dqn`内新建一个`ckpt`目录，放入上述模型文件
> - 在代码包`conf/configure_app.toml`中完成以下配置：
>
> ```yaml
> preload_model = true
> preload_model_dir = "agent_dqn/ckpt"
> preload_model_id = 40052
> ```
>
> 
>
> 完成上述步骤后，Aisrv和Learner将加载预训练模型以继续训练。

### 加载对手模型功能

在使用腾讯开悟平台时，部分项目支持从平台的模型管理中加载自定义模型。 用户可以在 `kaiwu.json` 文件中配置期望用作对手模型的模型 ID。智能体通过 `load_opponent_agent()` 函数完成对手模型的加载。

该功能允许智能体加载指定的网络结构和参数，并根据模型文件中的实现调用 agent 的`reset`、`predict`、`exploit`等方法。 对手模型作为固定水平的自定义模型，可以在 PVP 任务中用于训练评估，以反映训练过程中模型水平的提高。

以下是`load_opponent_agent()`函数示例：

```python
@load_opponent_agent_wrapper
def load_opponent_agent(self, id="1"):
    # Framework provides loading opponent agent function, no need to implement function content
    # 框架提供的加载对手模型功能，无需实现函数内容
    pass
```



**Parameters**

| 参数名 | 介绍                                                         |
| ------ | ------------------------------------------------------------ |
| **id** | string类型，平台模型管理的 模型id，使用 id 指定加载的模型文件 |

以下是kaiwu.json的配置示例：

```json
{
    "model_pool": [609, 608]
}
```



### 策略类型选择

本框架支持两种核心的训练范式：**同策略（On-Policy）**与**异策略（Off-Policy）**。此选择是训练流程的基础，将直接影响数据的使用方式、内存占用以及部分超参数的推荐设置。

| 特性     | On-Policy                                                    | Off-Policy                                                   |
| -------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| 数据来源 | 必须使用由当前正在训练的策略实时与环境交互产生的**新数据**。 | 可以使用**历史经验数据**，这些数据可能来自旧版本的策略或不同的探索策略。 |
| 数据使用 | **一次性为主**。数据在用于一次策略更新后通常被丢弃，以确保训练数据与当前策略的一致性。 | **可重复利用**。数据被存储在“经验回放缓冲区”中，可被多次采样用于训练。 |
| 主要优势 | **训练稳定**，理论收敛性有保障，行为容易理解。               | **数据效率高**，可充分利用每一次交互获得的数据，更适合现实世界中交互成本高的场景。 |

#### 配置参数详解

在代码包`conf/configure_app.toml`中，通过以下参数进行设置：

```yaml
# Training paradigm: on-policy or off-policy
# 训练时采用on-policy, off-policy
algorithm_on_policy_or_off_policy = "on-policy"

# Model dump frequency (steps)
# Interval (in steps) for saving model parameter files.
# on_policy: set to 1 for optimal training.
# off_policy: recommend 1000 to reduce save frequency.
# 训练间隔多少步输出模型参数文件, on_policy时设置为1训练效果最好，off_policy时建议设置为1000, 减少模型保存频率
dump_model_freq = 1
```
