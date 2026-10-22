# 精简接口说明

## 调用流程

接口内部按以下顺序工作：读取或接收配置，检查输入，构造静态地图，把高优先级车辆轨迹写入 X-Y-T 资源块，然后调用同一套 V-Hybrid A* 搜索器。搜索成功后，使用 Day3 车辆运动学模型重新播放控制序列，生成固定时间采样轨迹；失败则立即返回安全停车轨迹。

## 输入约定

如果配置已经在启动阶段读取，可以把运行时配置作为第二个参数传入：

```matlab
runtime = load_summon_runtime_config();
plan = plan_summon_trajectory(request,runtime);
```

`start_state` 和 `goal_state` 支持 `[x y yaw]`、`[x y yaw v]` 和 `[x y yaw v t]`，接口会补齐缺省的速度和时间。为保持输出时间语义一致，起点和目标时间必须为 `0`。速度字段始终是非负大小，行驶方向单独由 `gear` 表示，不能用负速度代替倒车。

可选的 `output_dt_s` 默认是 `0.1`，`max_output_time_s` 默认是 `60`，最大时域必须是采样周期的整数倍。若调用方提供 `map`，它必须是 `summon_map` 生成的结构；也可以传 `map_config` 或 `static_obstacles` 让接口构造地图。

动态障碍物必须带时间。推荐使用：

```matlab
request.higher_priority_trajectories = struct( ...
    'id','vehicle_002', ...
    'trajectory',[x(:),y(:),yaw(:),speed(:),time(:)]);
```

## 输出约定

`x/y/yaw/time` 是列向量，长度默认为 601。`speed` 是非负值，`gear=1` 表示前进，`gear=-1` 表示倒车，`gear=0` 只用于无有效轨迹或安全停车。换挡等待由 `is_gear_change` 标记，当前配置为 `0.5 s`。成功轨迹的最后速度满足终点停车约束。

`computation_time_s` 是从接口开始到输出生成完成的墙钟时间。`search_statistics` 只对外暴露展开节点、生成节点、各类拒绝计数、反向距离、换挡次数等摘要；详细搜索节点、占用栅格和图像均不属于部署接口。

## 失败处理

任何输入、配置、静态碰撞、动态碰撞、节点上限或超时失败都会设置 `valid=false`。接口仍返回与成功时相同长度的时间轨迹，位置保持在已校验的真实起点，速度和加速度为零，挡位为零。控制器应据此执行安全停车，并依据 `error_code/message/diagnostics` 决定是否人工处理或重新发起规划。

## Simulink 预留

JSON 读取只适合启动阶段。MATLAB Function Block 内应使用已解析的结构体输入，调用入口的第二个 `runtime` 参数，避免在块执行期间调用 `fileread/jsondecode`。当前内部使用动态 MATLAB 数据结构，已完成 MATLAB 解释执行接口，但尚未声明支持 MATLAB Coder；需要生成 C/C++ 代码时，应先把 Open/Closed 集合、节点数组和资源地图改为固定尺寸数组并重新验证。
