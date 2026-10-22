function [collision, report] = check_trajectory_conflict(trajectory, obstacles, vehicle, planner, st_map, options)
%CHECK_TRAJECTORY_CONFLICT 检查候选轨迹段与高优先级轨迹的绝对时空冲突。
% 输入：候选path/Nx5、动态障碍列表、车辆/配置、可选Day5图和诊断选项。
% 输出：碰撞/资源冲突、来源/时间/位置、采样净距及保守下界等报告。
% 规划可关闭距离采样，但必须检查完整候选车身扫掠与动态资源；最终验收开启。
if nargin < 5, st_map = []; end
if nargin < 6, options = struct(); end
collect = true;
if isfield(options,'collect_distance_samples'), collect = options.collect_distance_samples; end
states = day6_trajectory_states(trajectory);
report = struct('collision',false,'reason','none','minimum_distance_m',NaN, ...
    'minimum_distance_lower_bound_m',NaN,'conflict_time_s',NaN, ...
    'conflict_position_xy',[NaN,NaN],'obstacle_id','', ...
    'resource_conflict',false,'physical_collision',false,'rectangle_collision',false,'checked_samples',0, ...
    'resource_indices',zeros(0,1),'resources_checked',0,'has_intermediate_conflict',false, ...
    'distance_evaluated',false,'distance_samples',zeros(0,2), ...
    'filter_statistics',struct('circle_positive',0,'aabb_positive',0,'rectangle_positive',0, ...
    'circle_false_positive',0,'aabb_false_positive',0));
collision = false;
extra = planner.day6.minimum_safety_distance_m/2;
if ~isempty(st_map), extra = max(extra,st_map.config.safety_distance_m); end
footprint = inflate_vehicle_occupancy(vehicle,extra);
if ~isempty(st_map)
    context_ids = cell(1,numel(obstacles));
    if ~isempty(obstacles), context_ids = {obstacles.id}; end
    if ~isfield(st_map,'dynamic_context_ids') || ~isequal(st_map.dynamic_context_ids,context_ids)
        st_map = prepare_dynamic_context(st_map,obstacles);
    end
    [~,inside] = st_resource_index(st_map,states(:,[1,2,5]));
    outside = any(~inside) || footprintOutside(states,st_map,footprint);
    indices = zeros(0,1);
    if ~outside && possibleResourceOverlap(states,st_map,footprint)
        [indices,outside,report.resources_checked] = day6_resource_blocks(states,st_map,footprint,~collect);
    end
    if outside
        collision = true; report.collision = true; report.resource_conflict = true;
        report.reason = 'outside_time_space_bounds';
        report.conflict_time_s = states(end,5); report.conflict_position_xy = states(end,1:2);
        return;
    end
    hits = indices(st_map.dynamic_occupancy(indices));
    if ~isempty(hits)
        collision = true; report.resource_conflict = true; report.reason = 'dynamic_resource_block';
        report.resource_indices = hits;
        [ix,iy,it] = ind2sub([st_map.nx,st_map.ny,st_map.nt],hits(1));
        report.conflict_time_s = max(states(1,5),min(states(end,5),st_map.t_coords(it)));
        report.conflict_position_xy = [st_map.x_coords(ix),st_map.y_coords(iy)];
        owner = double(st_map.dynamic_owner_index(hits(1)));
        if owner>0, report.obstacle_id = st_map.owners(owner).id; end
    end
    % 在完整且包含所有高优先级车的资源图中，无动态资源重叠即安全。
    if ~collect
        report.collision = collision;
        report.has_intermediate_conflict = collision && ...
            report.conflict_time_s > states(1,5)+1e-9 && report.conflict_time_s < states(end,5)-1e-9;
        return;
    end
end
if isempty(obstacles)
    report.distance_evaluated = collect; report.minimum_distance_m = Inf;
    report.minimum_distance_lower_bound_m = Inf; report.collision = collision;
    return;
end
% 时间节点、减速中点与均匀采样都保留，不能只比较两个端点。
duration = states(end,5)-states(1,5);
step = planner.day6.collision_sample_step_s;
samples = unique([states(:,5).',linspace(states(1,5),states(end,5),max(1,ceil(duration/step))+1)]);
ego = struct('states',states,'hold_end',false);
[ego_states,~] = predict_vehicle_pose(ego,samples);
if isstruct(trajectory) && all(isfield(trajectory,{'acceleration','steering_angle'}))
    ego_states = replayEgo(trajectory,states,samples,vehicle,planner);
end
minimum = inf(size(samples));
maximum_uncertainty = 0;
for n = 1:numel(obstacles)
    [other_states,active] = predict_vehicle_pose(obstacles(n),samples);
    other = obstacles(n).vehicle_config;
    relative_bound = cornerRate(states,footprint)+cornerRate(obstacles(n).states,obstacles(n).footprint);
    padding = relative_bound*step/4;
    maximum_uncertainty = max(maximum_uncertainty,relative_bound*step/2);
    % 单一时刻没有采样间运动区间，诊断必须用该时刻原安全矩形。
    % 不能把高车整条轨迹的最高速误加到静止起点，混淆实际安全冲突和资源冲突。
    if duration == 0, padding = 0; maximum_uncertainty = 0; end
    safety = planner.day6;
    % 两车各加半份运动不确定半径；矩形包含采样间连续运动。
    safety.sweep_padding_m = padding;
    for k = 1:numel(samples)
        if ~active(k), continue; end
        [hit,detail] = check_vehicle_vehicle_collision(ego_states(k,:),other_states(k,:),vehicle,other,safety);
        minimum(k) = min(minimum(k),detail.minimum_distance_m);
        report.checked_samples = report.checked_samples+1;
        report.filter_statistics.circle_positive = report.filter_statistics.circle_positive+double(detail.circle_overlap);
        report.filter_statistics.aabb_positive = report.filter_statistics.aabb_positive+double(detail.aabb_overlap);
        report.filter_statistics.rectangle_positive = report.filter_statistics.rectangle_positive+double(hit);
        report.filter_statistics.circle_false_positive = report.filter_statistics.circle_false_positive+double(detail.circle_overlap && ~hit);
        report.filter_statistics.aabb_false_positive = report.filter_statistics.aabb_false_positive+double(detail.aabb_overlap && ~hit);
        report.physical_collision = report.physical_collision || detail.raw_rectangle_overlap;
        if hit && ~report.rectangle_collision
            report.rectangle_collision = true;
            if ~collision
                report.reason = 'vehicle_rectangle_swept'; report.conflict_time_s = samples(k);
                report.conflict_position_xy = (ego_states(k,1:2)+other_states(k,1:2))/2;
                report.obstacle_id = obstacles(n).id;
            end
            collision = true;
        end
    end
end
report.minimum_distance_m = min(minimum);
report.minimum_distance_lower_bound_m = max(0,report.minimum_distance_m-maximum_uncertainty);
report.distance_samples = [samples(:),minimum(:)];
report.distance_evaluated = true;
report.collision = collision;
report.has_intermediate_conflict = collision && report.conflict_time_s > states(1,5)+1e-9 && ...
    report.conflict_time_s < states(end,5)-1e-9;
end

function possible = possibleResourceOverlap(states,map,footprint)
%POSSIBLERESOURCEOVERLAP 保守包络仅排除不可能命中的远距离段，不替代矩形检测。
% 每段弧长上界补偿采样点间的后轴运动，圆半径包住任意航向下完整车身。
gap = 0;
if size(states,1) > 1
    gap = max(0.5*(states(1:end-1,4)+states(2:end,4)).*diff(states(:,5)));
end
halo = footprint.radius+gap+map.config.sweep_max_distance_m;
ego_bounds = [min(states(:,1))-halo,max(states(:,1))+halo, ...
    min(states(:,2))-halo,max(states(:,2))+halo];
first = max(1,floor((states(1,5)-map.t_min)/map.dt)+1);
last = min(map.nt,floor((states(end,5)-map.t_min)/map.dt)+1);
b = map.dynamic_layer_bounds(first:last,:);
possible = any(b(:,1) <= ego_bounds(2) & b(:,2) >= ego_bounds(1) & ...
    b(:,3) <= ego_bounds(4) & b(:,4) >= ego_bounds(3));
end

function outside = footprintOutside(states,map,footprint)
%FOOTPRINTOUTSIDE 车身四角须位于空间范围内，不能仅检查后轴参考点。
c = cos(states(:,3)); s = sin(states(:,3));
center_x = states(:,1)+footprint.center_offset*c;
center_y = states(:,2)+footprint.center_offset*s;
extent_x = footprint.length/2*abs(c)+footprint.half_width*abs(s);
extent_y = footprint.length/2*abs(s)+footprint.half_width*abs(c);
outside = any(center_x-extent_x < map.x_min | center_x+extent_x >= map.x_max | ...
    center_y-extent_y < map.y_min | center_y+extent_y >= map.y_max);
end

function rate = cornerRate(states,footprint)
%CORNERRATE 给定插值轨迹最远车角的最大运动速率上界(m/s)。
if size(states,1) < 2, rate = 0; return; end
dt = diff(states(:,5));
linear = hypot(diff(states(:,1)),diff(states(:,2)))./dt;
angular = abs(diff(states(:,3)))./dt;
% 加减速重放的车角速率在段端可大于段均值；按端速度/均速放大角速率。
% 公共接口速度是大小；即使倒车，运动不确定半径也必须为非负。
mean_speed = 0.5*(abs(states(1:end-1,4))+abs(states(2:end,4)));
peak_speed = max(abs(states(1:end-1,4)),abs(states(2:end,4)));
factor = ones(size(mean_speed));
moving = mean_speed > 0;
factor(moving) = max(1,peak_speed(moving)./mean_speed(moving));
rate = max([linear;abs(states(:,4))])+footprint.radius*max(angular.*factor);
end

function states_out = replayEgo(trajectory,states,times,vehicle,planner)
%REPLAYEGO 对带控制的自车边按既有运动学重放，避免用位姿线性插值代替转弯。
% gear=+1/-1 为离散档位，v 保持非负；缺少 gear 的旧轨迹按前进重放。
states_out = zeros(numel(times),5);
gears = ones(size(states,1),1);
if isfield(trajectory,'gear'), gears = trajectory.gear(:); end
for k = 1:numel(times)
    index = find(states(:,5) <= times(k),1,'last');
    if index == size(states,1) || times(k) == states(index,5)
        states_out(k,:) = states(index,:); continue;
    end
    if gears(index) ~= gears(index+1) && ...
            (states(index,4) > 1e-9 || states(index+1,4) > 1e-9 || ...
            abs(trajectory.acceleration(index+1)) > 1e-9)
        error('DynamicCollision:InvalidTrajectory','换挡边必须停稳且加速度为零。');
    end
    [state,~,~,valid] = summon_vehicle_dynamic_gear( ...
        states(index,:),trajectory.steering_angle(index+1),trajectory.acceleration(index+1), ...
        times(k)-states(index,5),vehicle,planner,gears(index+1));
    if ~valid, error('DynamicCollision:InvalidTrajectory','轨迹控制无法重放。'); end
    states_out(k,:) = state;
end
end
