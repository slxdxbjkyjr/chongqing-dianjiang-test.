# 召集阶段路径规划精简版

本目录是 Day1–Day6 研究原型的独立部署副本。它保留单车 V-Hybrid A*、车辆运动学、静态碰撞、X-Y-T 资源块和顺序动态碰撞规划，去掉演示脚本、绘图、动画以及搜索过程文件。原始 APA 工程没有修改。

工作人员只需要调用一个入口：

```matlab
plan = plan_summon_trajectory(request);
```

若配置已在启动阶段解析并缓存，也可以显式传入第二个参数：

```matlab
runtime = load_summon_runtime_config();
plan = plan_summon_trajectory(request,runtime);
```

`request` 至少包含两个字段：

```matlab
request.start_state = [x, y, yaw, v, t];
request.goal_state  = [x, y, yaw, v, t];
```

坐标参考点是后轴中心，`x/y` 单位为 m，`yaw` 为 rad，`v` 为非负 m/s，`t` 为 s。一次性召集规划要求起点和目标状态的时间均为 `0`；输出时间轴从 `0` 开始。`goal_state(4)` 通常设置为 `0`，表示到达召集点时停车。

实际地图可以通过 `request.map_config`、`request.static_obstacles` 或预先由 `summon_map` 生成的 `request.map` 传入。高优先级车辆的带时间轨迹通过 `request.higher_priority_trajectories` 或 `request.dynamic_obstacles` 传入，支持 `N×5 [x y yaw v t]` 数组、path 结构体、cell 和带 `id/trajectory/states` 的结构体。

成功时 `plan.valid=true`，主要输出为：

- `x/y/yaw/time`：固定间隔的轨迹，默认 `0.1 s`，默认时域 `60 s`，共 601 个点；
- `speed`：非负速度大小；`v` 是同一字段的别名；
- `gear`：`+1` 前进，`-1` 倒车，`0` 表示安全停车或无有效挡位；
- `acceleration`、`steering_angle`、`is_gear_change`：控制器需要的运动学信息；
- `travelled_distance`：相邻采样点的带挡位行驶距离；
- `goal_reached_time_s`、`minimum_clearance_m`、`computation_time_s`：结果和性能诊断；
- `search_statistics`、`diagnostics`：只保留简要计数和失败原因，不暴露搜索节点、Open/Closed 集合或占用栅格。

规划失败时 `plan.valid=false`，返回非零 `error_code` 和中文 `message`，轨迹仍为固定长度的安全停车输出：位置保持在真实起点、速度全为零、`gear=0`。底盘接口自行处理停车挡位和制动。

常见错误码为：`100` 输入不合法，`101` 配置不合法，`2/3` 起点或终点静态碰撞，`4` Open 集合耗尽，`5` 节点上限，`6` 搜索超时，`7` 终端状态不满足，`8` 运动学验收失败，`9/10` 动态碰撞，`11` 超出时空范围，`102` 接口异常。

MATLAB Function Block 的推荐接入方式是：启动阶段用 `load_summon_runtime_config` 读取 JSON，将解析后的 `vehicle_config`、`map_config`、`planner_config` 缓存在模型工作区；运行调用用第二个参数传入该配置，只更新状态、地图和高优先级轨迹。当前内部搜索器仍使用 `containers.Map` 和动态结构体，尚未完成 MATLAB Coder 代码生成验证，因此本目录当前用于 MATLAB/Simulink 原型验证，部署到代码生成目标前还需要单独做 Coder 兼容性改造。

当前搜索器是召集前一次性规划原型，实测复杂 Day6 场景可能需要秒级到十几秒。它不适合作为行驶中的高频重规划器；如需周期性重规划，应先做搜索器性能重构和 C++/MEX 评估。

回归测试位于 `tests/test_runtime_interface.m`，可在 MATLAB 中执行：

```matlab
results = runtests('tests/test_runtime_interface.m');
disp(results);
```
