function [success, connection_nodes, statistics] = vhybrid_goal_connection( ...
    parent_node, goal_state, map, vehicle_config, planner_config, st_map, higher_priority_trajectories)
%VHYBRID_GOAL_CONNECTION 用真实 Dubins 控制段和既有运动学连接终点。
% 输入：父节点 [x,y,theta,v,t]、目标状态、静态地图和车辆/规划配置。
% 输出：success、父节点之后的连接节点、候选/碰撞/约束裁剪统计。
% 逻辑：枚举六类圆弧/直线组合，按最小转弯半径限制曲率；在弧长上
% 构造加速/匀速/减速剖面。每个点都由 summon_vehicle_dynamic 积分，
% 在几何段边界与速度相位边界切步。禁止直接赋值目标位置或航向。

cfg = planner_config.vhybrid;
dyn = planner_config.dynamics;
if nargin < 6, st_map = []; end
if nargin < 7, higher_priority_trajectories = []; end
has_context = ~isempty(st_map) || ~isempty(higher_priority_trajectories);
success = false;
connection_nodes = repmat(vhybrid_node(), 0, 1);
statistics = struct('candidates', 0, 'collision_pruned', 0, ...
    'constraint_pruned', 0, 'terminal_speed', NaN, 'connection_length_m', NaN, ...
    'dynamic_pruned',0,'resource_pruned',0,'reason_counts',struct(), ...
    'conflict_examples',{{}},'geometric_candidates',0,'boundary_geometry_pruned',0);
start = [parent_node.x,parent_node.y,parent_node.theta,parent_node.v,parent_node.t];
gear = 1;
if isfield(parent_node,'gear'), gear = parent_node.gear; end
if gear < 0 && isfield(cfg,'max_reverse_speed_mps')
    dyn.v_max_mps = min(dyn.v_max_mps,cfg.max_reverse_speed_mps);
end
goal_state = double(goal_state(:).');
if hypot(start(1)-goal_state(1),start(2)-goal_state(2)) > cfg.goal_connection_distance_m
    return;
end
delta_max = min(vehicle_config.steering.maximum_steer_rad, dyn.delta_max_rad);
radius = vehicle_config.dimensions_m.wheelbase / tan(delta_max);
% 倒车的运动切向比车身航向多 pi；同挡位连接不隐式换挡。
motion_start = start(1:3); motion_goal = goal_state(1:3);
if gear<0
    motion_start(3) = motion_start(3)+pi; motion_goal(3) = motion_goal(3)+pi;
end
curves = vhybrid_dubins_candidates(motion_start,motion_goal,radius);
% 采样的是连接段巡航速度，终端速度始终为目标停车约束。
speeds = unique([cfg.goal_connection_speed_samples_mps(:).', ...
    cfg.reference_speed_mps, start(4), cfg.terminal_speed_mps]);
speeds = sort(speeds, 'descend');
for curve_id = 1:numel(curves)
    curve = curves(curve_id);
    statistics.geometric_candidates = statistics.geometric_candidates+1;
    % 先用车身角点完整圆弧极值证明是否必定越界，再尝试速度相位。
    % 这避免反复用不同速度积分同一条必不可行的几何曲线。
    [outside,~] = vhybrid_curve_boundary_check(start,curve,radius,map,vehicle_config,gear);
    if outside
        statistics.boundary_geometry_pruned = statistics.boundary_geometry_pruned+1;
        key = 'analytic_boundary_collision';
        if ~isfield(statistics.reason_counts,key), statistics.reason_counts.(key) = 0; end
        statistics.reason_counts.(key) = statistics.reason_counts.(key)+1;
        continue;
    end
    for cruise_speed = speeds
        statistics.candidates = statistics.candidates + 1;
        [phases, feasible] = speedPhases(sum(curve.lengths),start(4), ...
            cfg.terminal_speed_mps,cruise_speed,dyn);
        if ~feasible
            statistics.constraint_pruned = statistics.constraint_pruned + 1;
            continue;
        end
        [trial, reason] = integrateCurve(parent_node, goal_state, curve, phases, ...
            delta_max, map, vehicle_config, planner_config);
        if strcmp(reason,'collision')
            statistics.collision_pruned = statistics.collision_pruned + 1;
            % 静态碰撞仅依赖几何，对同一曲线换速度无法绕开障碍物。
            break;
        elseif ~strcmp(reason,'ok')
            statistics.constraint_pruned = statistics.constraint_pruned + 1;
            continue;
        end
        if has_context
            % 终点解析连接与普通扩展接受同一个连续碰撞验收，不能绕过
            % 资源块。动态冲突只拒绝当前速度，相同曲线仍可换速度重试。
            trajectory = connectionTrajectory(parent_node,trial);
            [collision,report] = check_trajectory_conflict(trajectory, ...
                higher_priority_trajectories,vehicle_config,planner_config,st_map, ...
                struct('collect_distance_samples',false));
            if ~collision && ~isempty(higher_priority_trajectories) && ...
                    isfield(planner_config,'day6') && planner_config.day6.goal_hold_enabled
                hold_trajectory = goalHoldTrajectory(trajectory,st_map,planner_config);
                [collision,report] = check_trajectory_conflict(hold_trajectory, ...
                    higher_priority_trajectories,vehicle_config,planner_config,st_map, ...
                    struct('collect_distance_samples',false));
            end
            if collision
                statistics.collision_pruned = statistics.collision_pruned + 1;
                statistics.dynamic_pruned = statistics.dynamic_pruned + double( ...
                    report.rectangle_collision || report.physical_collision || ...
                    (report.resource_conflict && ~strcmp(report.reason,'outside_time_space_bounds')));
                statistics.resource_pruned = statistics.resource_pruned + double(report.resource_conflict);
                key = matlab.lang.makeValidName(char(report.reason));
                if ~isfield(statistics.reason_counts,key), statistics.reason_counts.(key) = 0; end
                statistics.reason_counts.(key) = statistics.reason_counts.(key)+1;
                if numel(statistics.conflict_examples) < planner_config.day6.max_conflict_examples
                    statistics.conflict_examples{end+1} = report;
                end
                continue;
            end
        end
        connection_nodes = trial;
        statistics.terminal_speed = trial(end).v;
        statistics.connection_length_m = sum(curve.lengths);
        success = true;
        return;
    end
end

function trajectory = connectionTrajectory(parent,nodes)
%CONNECTIONTRAJECTORY 将真实模型积分的连接节点转换为统一带控制轨迹。
trajectory = struct('x',[parent.x;vertcat(nodes.x)], ...
    'y',[parent.y;vertcat(nodes.y)],'theta',[parent.theta;vertcat(nodes.theta)], ...
    'v',[parent.v;vertcat(nodes.v)],'t',[parent.t;vertcat(nodes.t)], ...
    'gear',[parent.gear;vertcat(nodes.gear)], ...
    'is_gear_change',[false;vertcat(nodes.is_gear_change)], ...
    'acceleration',[0;vertcat(nodes.acceleration)], ...
    'steering_angle',[0;vertcat(nodes.steering_angle)]);
end

function trajectory = goalHoldTrajectory(path,st_map,planner)
%GOALHOLDTRAJECTORY 检查终点停车后直到地图末时间层的资源占用。
if isempty(st_map), horizon = planner.st_occupancy.t_max;
else, horizon = st_map.t_max; end
last_time = horizon-max(1e-9,16*eps(horizon));
trajectory = struct('x',path.x(end),'y',path.y(end),'theta',path.theta(end), ...
    'v',path.v(end),'t',path.t(end),'acceleration',0,'steering_angle',0, ...
    'gear',path.gear(end),'is_gear_change',false);
if last_time > path.t(end)
    trajectory.x(2,1) = path.x(end); trajectory.y(2,1) = path.y(end);
    trajectory.theta(2,1) = path.theta(end); trajectory.v(2,1) = 0;
    trajectory.t(2,1) = last_time;
    trajectory.acceleration(2,1) = 0; trajectory.steering_angle(2,1) = 0;
    trajectory.gear(2,1) = path.gear(end); trajectory.is_gear_change(2,1) = false;
end
end
end


function [phases, feasible] = speedPhases(distance, v0, vf, cap, dyn)
%SPEEDPHASES 三角形/梯形速度剖面，用 v²-v0²=2*a*s 保证路程一致。
% 相位字段：[持续时间, 加速度, 相位末速度]，终端 vf 是硬约束。
phases = zeros(0,3); feasible = false;
ap = dyn.a_max_mps2; ab = -dyn.a_min_mps2;
if distance <= 1e-10 || ap <= 0 || ab <= 0 || cap <= 0 || ...
        cap < max(v0,vf) || cap > dyn.v_max_mps || min(v0,vf) < dyn.v_min_mps
    return;
end
peak = min(cap,sqrt((2*ap*ab*distance+ab*v0^2+ap*vf^2)/(ap+ab)));
if peak < max(v0,vf)-1e-10
    return; % 剩余距离不足以在约束内停车；不能人为截断速度。
end
peak = max(peak,max(v0,vf));
ta = (peak-v0)/ap; tb = (peak-vf)/ab;
cruise_distance = distance-(peak^2-v0^2)/(2*ap)-(peak^2-vf^2)/(2*ab);
tc = max(0,cruise_distance)/peak;
phases = [ta,ap,peak; tc,0,peak; tb,-ab,vf];
phases = phases(phases(:,1)>1e-12,:);
feasible = ~isempty(phases);
end

function [nodes, reason] = integrateCurve(parent, goal, curve, phases, ...
    delta_max, map, vehicle, planner)
%INTEGRATECURVE 以相位/圆弧边界为事件切步，积分同一个自行车模型。
cfg = planner.vhybrid; dyn = planner.dynamics;
nodes = repmat(vhybrid_node(),0,1); reason = 'constraint';
state = [parent.x,parent.y,parent.theta,parent.v,parent.t];
gear = parent.gear;
previous = parent; segment = 1; phase = 1;
segment_remaining = curve.lengths(1); time_remaining = phases(1,1);
while segment <= 3 && phase <= size(phases,1)
    if segment_remaining <= 1e-10 && segment < 3
        segment = segment+1;
        if segment <= 3, segment_remaining = curve.lengths(segment); end
        continue;
    end
    % 相位减法可留下机器精度的正残余；此时再积分会使t+dt==t。
    % 仅跳过不可分辨的相位尾部，末速度仍由真实积分与末状态容差验收。
    phase_time_tolerance = 64*eps(max(1,abs(state(5))+time_remaining));
    if time_remaining <= phase_time_tolerance
        phase = phase+1;
        if phase <= size(phases,1), time_remaining = phases(phase,1); end
        continue;
    end
    % 根据剩余相位时间修正舍入误差；不修改状态，不超过加速度边界。
    acceleration = (phases(phase,3)-state(4))/time_remaining;
    acceleration = max(dyn.a_min_mps2,min(dyn.a_max_mps2,acceleration));
    steering = gear*curve.types(segment)*delta_max;
    dt = min(cfg.goal_connection_step_s,time_remaining);
    ds_limit = min(segment_remaining,cfg.collision_check_step_m);
    if segment == 3 && segment_remaining <= 1e-10
        % 最末段不能因弧长舍入提前结束制动，仍积分完整相位直到 v=0。
        ds_limit = cfg.collision_check_step_m;
    end
    possible_distance = state(4)*dt+0.5*acceleration*dt^2;
    if possible_distance > ds_limit
        % 稳定解法，避免减速时求根的数值抵消。
        end_speed = sqrt(max(0,state(4)^2+2*acceleration*ds_limit));
        dt = min(dt,2*ds_limit/(state(4)+end_speed));
    end
    [next,ds,~,valid] = summon_vehicle_dynamic_gear(state,steering,acceleration,dt, ...
        vehicle,planner,gear);
    if ~valid || abs(ds) <= 0 || next(5) <= state(5)
        return;
    end
    if day6_static_collision_check(next(1:3),map,vehicle)
        reason = 'collision'; return;
    end
    [g,h] = vhybrid_cost(previous,next,goal,planner,ds,steering,gear,false);
    indices = [floor((next(1)-map.bounds(1))/cfg.xy_grid_resolution_m), ...
        floor((next(2)-map.bounds(2))/cfg.xy_grid_resolution_m), ...
        round((mod(next(3)+pi,2*pi))/cfg.yaw_grid_resolution_rad), ...
        round(next(4)/cfg.velocity_grid_resolution_mps),round(next(5)/cfg.time_grid_resolution_s)];
    node = vhybrid_node(next,acceleration,steering,previous.node_id,g,h,indices,0,gear,false);
    node.direction = gear; node.travelled_distance = ds;
    nodes(end+1,1) = node; %#ok<AGROW>
    previous = node; state = next;
    segment_remaining = segment_remaining-abs(ds); time_remaining = time_remaining-dt;
end
% 验收模型积分的真实末状态，绝不将节点替换成目标。
if ~isempty(nodes) && hypot(state(1)-goal(1),state(2)-goal(2)) <= cfg.goal_position_tolerance_m && ...
        abs(mod(state(3)-goal(3)+pi,2*pi)-pi) <= cfg.goal_heading_tolerance_rad && ...
        abs(state(4)-cfg.terminal_speed_mps) <= cfg.goal_speed_tolerance_mps
    reason = 'ok';
end
end
