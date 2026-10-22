function plan = plan_summon_trajectory(request,runtime_override)
%PLAN_SUMMON_TRAJECTORY 召集阶段一次性单车/顺序多车路径规划入口。
% 输入request至少包含：
%   start_state - [x,y,yaw,v,t]，也可省略v/t（默认0）；后轴中心、m/rad。
%   goal_state  - [x,y,yaw,v,t]，目标v默认0，要求最终停车。
%   可选map/static_map、vehicle_config、planner_config、
%   higher_priority_trajectories或dynamic_obstacles、options。
% 输出plan只保留实车需要的固定时间轨迹和简要诊断：
%   valid/error_code/message、x/y/yaw/theta/speed/time/gear、
%   acceleration/steering_angle/is_gear_change/travelled_distance、
%   computation_time_s/search_statistics/minimum_clearance_m。
% 约定：输出时间默认0:0.1:60 s；speed为非负大小，gear=+1前进、-1倒车，
% 失败时speed全为0且gear为0，由底盘接口决定制动/挡位。
% 说明：该入口面向MATLAB解释执行和MATLAB/Simulink原型。当前内部搜索器
% 使用containers.Map和动态结构体，尚未完成MATLAB Coder代码生成验证。
% 可选runtime_override为load_summon_runtime_config返回的预解析配置，
% 适合在Simulink启动阶段缓存配置后传入，避免运行调用重复读取JSON。

timer_id = tic;
plan = emptyRuntimePlan();
plan.computation_time_s = NaN;
% 保存已校验的真实起点，供异常分支生成安全停车轨迹。
normalized = struct('start_state',zeros(1,5));
try
    ensureCorePath();
    if nargin >= 2 && ~isempty(runtime_override)
        runtime = runtime_override;
    elseif isstruct(request) && isscalar(request) && isfield(request,'runtime') && ...
            ~isempty(request.runtime)
        runtime = request.runtime;
    else
        runtime = load_summon_runtime_config();
    end
    [is_valid, normalized, request_code, request_message] = ...
        validate_plan_request(request,runtime);
    if ~is_valid
        plan = safeFailurePlan(plan,normalized,request_code,request_message,timer_id);
        return;
    end

    vehicle_config = normalized.vehicle_config;
    planner_config = normalized.planner_config;
    map = resolveMap(normalized,runtime.map_config);
    map = prepare_static_context(map,vehicle_config);
    [obstacles, st_map] = buildDynamicContext(normalized.higher_priority_trajectories, ...
        map,vehicle_config,planner_config);

    search_st_map = st_map;
    if isempty(obstacles)
        % 无高优先级车辆时不建立动态资源上下文，保留单车搜索的最低开销。
        search_st_map = [];
    end
    start_state = normalized.start_state;
    goal_state = normalized.goal_state;
    path = summon_vhybrid_astar(start_state,goal_state,map,vehicle_config,planner_config, ...
        search_st_map,obstacles,normalized.options);

    plan = publicPlanFromPath(plan,path,start_state,vehicle_config,planner_config, ...
        normalized.output_dt_s,normalized.max_output_time_s,timer_id);
catch exception
    plan = safeFailurePlan(plan,normalized,uint8(102), ...
        ['规划接口异常：',exception.message],timer_id);
    plan.diagnostics = struct('identifier',exception.identifier,'exception_message',exception.message);
end
end

function ensureCorePath()
%ENSURECOREPATH 让工作人员从任意当前目录调用唯一入口。
persistent path_ready
% MATLAB 会话中的工具可能重置搜索路径；仅依赖persistent标志会造成
% 核心函数已经不在路径上却不再addpath，因此同时检查一个核心入口。
if isempty(path_ready) || ~path_ready || exist('summon_map','file') ~= 2
    interface_dir = fileparts(mfilename('fullpath'));
    root_dir = fileparts(fileparts(interface_dir));
    core_dir = fullfile(root_dir,'matlab','core');
    if exist(core_dir,'dir') ~= 7
        error('Summon:MissingCore','找不到核心代码目录：%s。',core_dir);
    end
    addpath(core_dir,'-end');
    path_ready = true;
end
end

function map = resolveMap(request,map_config)
%RESOLVEMAP 解析预构造地图或配置地图，并支持调用方覆盖静态障碍物。
if isfield(request,'map') && ~isempty(request.map)
    map = request.map;
    return;
end
if isfield(request,'map_config') && ~isempty(request.map_config)
    map_config = request.map_config;
end
if isfield(request,'static_obstacles')
    map_config.static_obstacles = request.static_obstacles;
end
map = summon_map(map_config);
end

function [obstacles, st_map] = buildDynamicContext(inputs,map,vehicle,planner)
%BUILDDYNAMICCONTEXT 把高优先级轨迹写成车辆矩形的X-Y-T资源块。
obstacles = normalizeObstacles(inputs,vehicle,planner);
st_map = [];
if isempty(obstacles), return; end
st_map = st_occupancy_map(planner,map);
[st_map,~] = mark_static_obstacles(st_map,map,vehicle);
for k = 1:numel(obstacles)
    states = occupancyStates(obstacles(k).states,st_map.t_max);
    if isempty(states)
        error('Summon:DynamicTrajectory','高优先级轨迹%s没有有效的时空状态。',obstacles(k).id);
    end
    st_map = mark_trajectory_occupancy(st_map,states, ...
        obstacles(k).vehicle_config,obstacles(k).id);
    % 几何精确检查仍使用原始轨迹；资源块只使用严格半开时间范围的副本。
end
st_map = prepare_dynamic_context(st_map,obstacles);
end

function obstacles = normalizeObstacles(inputs,vehicle,planner)
%NORMALIZEOBSTACLES 接受path、Nx5数组、cell或带id/trajectory的结构。
obstacles = repmat(struct('id','','states',zeros(0,5),'vehicle_config',vehicle, ...
    'footprint',struct(),'hold_end',false),0,1);
if isempty(inputs), return; end
if iscell(inputs)
    items = inputs;
elseif isnumeric(inputs)
    items = {inputs};
elseif isstruct(inputs)
    if isscalar(inputs) && (isfield(inputs,'x') || isfield(inputs,'states') || isfield(inputs,'trajectory'))
        items = {inputs};
    else
        items = num2cell(inputs);
    end
else
    error('Summon:DynamicTrajectory','动态障碍物必须是path、Nx5、cell或结构体数组。');
end
for k = 1:numel(items)
    item = items{k};
    owner_id = sprintf('higher_priority_%03d',k);
    trajectory = item;
    item_vehicle = vehicle;
    if isstruct(item)
        if isfield(item,'id') && ~isempty(item.id), owner_id = char(item.id); end
        if isfield(item,'vehicle_config') && ~isempty(item.vehicle_config)
            item_vehicle = item.vehicle_config;
        end
        if isfield(item,'trajectory')
            trajectory = item.trajectory;
        elseif isfield(item,'states') && ~isfield(item,'x')
            trajectory = item.states;
        end
    end
    obstacle = dynamic_vehicle_obstacle(trajectory,item_vehicle,owner_id,planner);
    obstacles(end+1,1) = obstacle; %#ok<AGROW>
end
end

function states = occupancyStates(states,t_max)
%OCCUPANCYSTATES 将资源图终点转换为严格小于t_max的半开时间坐标。
states = double(states);
keep = states(:,5) < t_max;
last_valid = find(keep,1,'last');
if isempty(last_valid), states = zeros(0,5); return; end
if last_valid < size(states,1)
    limit = t_max-max(1e-9,16*eps(max(1,t_max)));
    if limit > states(last_valid,5)
        a = states(last_valid,:); b = states(last_valid+1,:);
        fraction = (limit-a(5))/(b(5)-a(5));
        clipped = a+(b-a)*fraction;
        clipped(5) = limit;
        states = [states(1:last_valid,:);clipped];
    else
        states = states(1:last_valid,:);
    end
else
    states = states(1:last_valid,:);
end
end

function plan = publicPlanFromPath(plan,path,start_state,vehicle,planner,dt,horizon,timer_id)
%PUBLICPLANFROMPATH 删除搜索过程字段，仅输出固定时间控制轨迹。
plan.valid = logical(path.valid);
plan.error_code = double(path.error_code);
plan.message = pathMessage(path.error_code,path);
plan.diagnostics = path.failure_diagnostics;
if path.valid
    % 固定输出时域必须覆盖完整搜索路径；如果只输出搜索路径的前半段，
    % 调用方会收到尚未到达目标且速度不为0的假成功结果。
    if path.t(end) > horizon + 1e-9
        plan = safeTrajectory(plan,start_state,dt,horizon);
        plan.valid = false;
        plan.error_code = 7;
        plan.message = sprintf('搜索路径需要%.3fs，超过输出时域%.3fs。',path.t(end),horizon);
        plan.valid_until = 0;
        plan.goal_reached_time_s = NaN;
        plan.diagnostics = struct('safe_stop',true,'reason','output_horizon_too_short', ...
            'required_time_s',path.t(end),'requested_horizon_s',horizon);
    else
        trajectory = resamplePath(path,vehicle,planner,dt,horizon);
        plan.x = trajectory.x; plan.y = trajectory.y;
        plan.yaw = trajectory.theta; plan.theta = trajectory.theta;
        plan.speed = trajectory.v; plan.v = trajectory.v;
        plan.time = trajectory.t; plan.t = trajectory.t;
        plan.gear = trajectory.gear;
        plan.acceleration = trajectory.acceleration;
        plan.steering_angle = trajectory.steering_angle;
        plan.is_gear_change = trajectory.is_gear_change;
        plan.travelled_distance = trajectory.travelled_distance;
        plan.valid_until = trajectory.t(end);
        plan.goal_reached_time_s = path.t(end);
    end
else
    plan = safeTrajectory(plan,start_state,dt,horizon);
    plan.valid_until = 0;
    plan.goal_reached_time_s = NaN;
end
plan.search_statistics = publicStatistics(path.search_statistics);
plan.minimum_clearance_m = NaN;
if isfield(path,'dynamic_validation') && isfield(path.dynamic_validation,'minimum_distance_lower_bound_m')
    plan.minimum_clearance_m = path.dynamic_validation.minimum_distance_lower_bound_m;
end
plan.computation_time_s = toc(timer_id);
end

function trajectory = resamplePath(path,vehicle,planner,dt,horizon)
%RESAMPLEPATH 用同一车辆运动学模型生成固定时间输出，不线性插值转弯。
times = (0:dt:horizon).';
n = numel(times);
trajectory = struct('x',zeros(n,1),'y',zeros(n,1),'theta',zeros(n,1), ...
    'v',zeros(n,1),'t',times,'gear',ones(n,1),'acceleration',zeros(n,1), ...
    'steering_angle',zeros(n,1),'is_gear_change',false(n,1), ...
    'travelled_distance',zeros(n,1));
path_times = double(path.t(:));
path_gears = ones(numel(path_times),1);
if isfield(path,'gear'), path_gears = double(path.gear(:)); end
shift_times = zeros(0,1);
if isfield(path,'is_gear_change')
    flags = logical(path.is_gear_change(:));
    shift_times = path_times(flags);
end
for k = 1:n
    target = times(k);
    if target <= path_times(1)
        state = [path.x(1),path.y(1),path.theta(1),path.v(1),target];
        selected_gear = path_gears(1);
        acceleration = 0; steering = 0;
    elseif target >= path_times(end)
        if target > path_times(end)
            state = [path.x(end),path.y(end),path.theta(end),0,target];
            selected_gear = path_gears(end); acceleration = 0; steering = 0;
        else
            state = [path.x(end),path.y(end),path.theta(end),path.v(end),target];
            selected_gear = path_gears(end); acceleration = path.acceleration(end); steering = path.steering_angle(end);
        end
    else
        left = find(path_times < target,1,'last');
        local_time = target-path_times(left);
        start = [path.x(left),path.y(left),path.theta(left),path.v(left),path.t(left)];
        selected_gear = path_gears(left+1);
        acceleration = path.acceleration(left+1);
        steering = path.steering_angle(left+1);
        [state,~,~,valid] = summon_vehicle_dynamic_gear(start,steering,acceleration, ...
            local_time,vehicle,planner,selected_gear);
        if ~valid
            error('Summon:OutputResample','搜索路径无法由车辆运动学模型重放。');
        end
        state(5) = target;
    end
    trajectory.x(k) = state(1); trajectory.y(k) = state(2);
    trajectory.theta(k) = state(3); trajectory.v(k) = max(0,state(4));
    trajectory.gear(k) = selected_gear;
    trajectory.acceleration(k) = acceleration;
    trajectory.steering_angle(k) = steering;
    trajectory.is_gear_change(k) = any(abs(shift_times-target) <= 64*eps(max(1,target)));
end
if n > 1
    trajectory.travelled_distance(2:end) = trajectory.gear(2:end).*hypot( ...
        diff(trajectory.x),diff(trajectory.y));
end
end

function statistics = publicStatistics(stats)
%PUBLICSTATISTICS 保留工作人员定位性能和失败原因所需的统计。
names = {'elapsed_sec','expanded_nodes','generated_nodes','sampled_controls', ...
    'collision_pruned_nodes','constraint_pruned_nodes','closed_pruned_nodes', ...
    'open_peak','dynamic_pruned','resource_pruned','waiting_nodes', ...
    'reverse_nodes','gear_switch_nodes','gear_switch_pruned', ...
    'path_reverse_distance_m','path_gear_switches','path_gear_wait_time_s', ...
    'reason_counts'};
statistics = struct();
for k = 1:numel(names)
    if isfield(stats,names{k}), statistics.(names{k}) = stats.(names{k}); end
end
end

function plan = safeFailurePlan(plan,normalized,error_code,message,timer_id)
%SAFEFAILUREPLAN 失败统一输出零速度安全停车轨迹。
if isfield(normalized,'start_state') && numel(normalized.start_state) == 5
    start_state = normalized.start_state;
else
    start_state = zeros(1,5);
end
dt = 0.1; horizon = 60;
if isfield(normalized,'output_dt_s'), dt = normalized.output_dt_s; end
if isfield(normalized,'max_output_time_s'), horizon = normalized.max_output_time_s; end
plan = safeTrajectory(plan,start_state,dt,horizon);
plan.valid = false;
plan.error_code = double(error_code);
plan.message = message;
plan.valid_until = 0;
plan.goal_reached_time_s = NaN;
plan.search_statistics = struct('elapsed_sec',0,'expanded_nodes',0,'generated_nodes',0, ...
    'sampled_controls',0,'collision_pruned_nodes',0,'constraint_pruned_nodes',0, ...
    'closed_pruned_nodes',0,'open_peak',0,'dynamic_pruned',0,'resource_pruned',0, ...
    'waiting_nodes',0,'reverse_nodes',0,'gear_switch_nodes',0,'gear_switch_pruned',0, ...
    'path_reverse_distance_m',0,'path_gear_switches',0,'path_gear_wait_time_s',0, ...
    'reason_counts',struct());
plan.diagnostics = struct('safe_stop',true,'reason',message);
plan.computation_time_s = toc(timer_id);
end

function plan = safeTrajectory(plan,start_state,dt,horizon)
times = (0:dt:horizon).';
n = numel(times);
plan.x = repmat(start_state(1),n,1); plan.y = repmat(start_state(2),n,1);
plan.yaw = repmat(start_state(3),n,1); plan.theta = plan.yaw;
plan.speed = zeros(n,1); plan.v = plan.speed; plan.time = times; plan.t = times;
plan.gear = zeros(n,1); plan.acceleration = zeros(n,1); plan.steering_angle = zeros(n,1);
plan.is_gear_change = false(n,1); plan.travelled_distance = zeros(n,1);
plan.minimum_clearance_m = NaN;
end

function plan = emptyRuntimePlan()
plan = struct('valid',false,'error_code',double(102),'message','尚未执行规划。', ...
    'x',zeros(0,1),'y',zeros(0,1),'yaw',zeros(0,1),'theta',zeros(0,1), ...
    'speed',zeros(0,1),'v',zeros(0,1),'time',zeros(0,1),'t',zeros(0,1), ...
    'gear',zeros(0,1),'acceleration',zeros(0,1),'steering_angle',zeros(0,1), ...
    'is_gear_change',false(0,1),'travelled_distance',zeros(0,1), ...
    'valid_until',0,'goal_reached_time_s',NaN,'computation_time_s',NaN, ...
    'minimum_clearance_m',NaN,'search_statistics',struct(),'diagnostics',struct());
end

function message = pathMessage(error_code,path)
labels = {'成功','输入状态不合法','起点静态碰撞','终点静态碰撞', ...
    'Open集合耗尽','达到节点上限','达到搜索时间上限','终端状态未满足', ...
    '运动学验收失败','起点动态碰撞','终点动态碰撞','超过时空时间范围'};
index = double(error_code)+1;
if index >= 1 && index <= numel(labels)
    message = labels{index};
else
    message = sprintf('规划失败，错误码%d。',double(error_code));
end
if ~path.valid && isfield(path,'failure_diagnostics') && isfield(path.failure_diagnostics,'reason')
    message = [message,'：',char(path.failure_diagnostics.reason)];
end
end
