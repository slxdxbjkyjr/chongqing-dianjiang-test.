function [map, statistics] = mark_trajectory_occupancy(map, trajectory, vehicle_config, owner_id)
%MARK_TRAJECTORY_OCCUPANCY 插值并保守标记动态矩形的整个时间层扫掠。
% 输入：地图、Nx5[x,y,theta,v,t]或Day4 path、车辆配置、唯一外部车辆ID。
% 输出：新地图、块数/冲突数/采样数；动态占用仅在轨迹覆盖的时间块内。
% 坐标为后轴中心；yaw沿最短角差插值；对每个时间块切段并补偿采样间隙。
states = trajectoryStates(trajectory);
if any(states(:,5) < map.t_min | states(:,5) >= map.t_max)
    error('STOccupancy:TrajectoryOutsideTime','轨迹必须完全落在配置半开时间范围内。');
end
if any(states(:,1) < map.x_min | states(:,1) >= map.x_max | ...
        states(:,2) < map.y_min | states(:,2) >= map.y_max)
    error('STOccupancy:TrajectoryOutsideSpace','轨迹后轴参考点越出时空地图空间范围。');
end
footprint = inflate_vehicle_occupancy(vehicle_config,map.config.safety_distance_m);
times = states(:,5);
states(:,3) = unwrap(states(:,3));
bins = cell(map.nt,1);
statistics = struct('sampled_poses',0,'swept_segments',0,'max_sweep_padding_m',0, ...
    'marked_resource_blocks',0,'conflict_blocks',0,'new_blocks',0, ...
    'time_range',[times(1),times(end)],'end_policy',map.config.trajectory_end_policy);
% 原始节点含单点轨迹和恰位于时间边界的末点，按共用索引写入所属块。
for k = 1:size(states,1)
    [index,~] = st_resource_index(map,[states(k,1:2),times(k)]);
    cells = st_vehicle_cells(map,states(k,1:3),footprint,0);
    bins{index(3)} = [bins{index(3)};cells];
end
for k = 1:size(states,1)-1
    a = times(k); b = times(k+1);
    cuts = [a,map.t_edges(map.t_edges > a & map.t_edges < b),b];
    for j = 1:numel(cuts)-1
        ta = cuts(j); tb = cuts(j+1);
        first = interpolatedState(states(k,:),states(k+1,:),ta);
        last = interpolatedState(states(k,:),states(k+1,:),tb);
        motion_bound = hypot(last(1)-first(1),last(2)-first(2))+ ...
            footprint.radius*abs(last(3)-first(3));
        count = max([1,ceil((tb-ta)/map.config.sweep_max_dt_s), ...
            ceil(motion_bound/map.config.sweep_max_distance_m)]);
        if isfield(map.config,'max_sweep_samples_per_segment') && ...
                count > map.config.max_sweep_samples_per_segment
            error('STOccupancy:SamplingLimit','单段扫掠采样数超过配置上限。');
        end
        % 每个中间车角离最近样本的距离<=motion_bound/(2*count)。
        % 沿局部纵/横轴各加该值包含这个欧氏球，覆盖连续插值扫掠。
        padding = motion_bound/(2*count);
        [index,~] = st_resource_index(map,[first(1:2),(ta+tb)/2]);
        it = index(3);
        sample_times = linspace(ta,tb,count+1);
        center_time = map.t_coords(it);
        if center_time >= ta && center_time <= tb
            sample_times = unique([sample_times,center_time]);
        end
        hits = cell(numel(sample_times),1);
        for sample = 1:numel(sample_times)
            state = interpolatedState(states(k,:),states(k+1,:),sample_times(sample));
            hits{sample} = st_vehicle_cells(map,state(1:3),footprint,padding);
        end
        bins{it} = [bins{it};vertcat(hits{:})];
        statistics.sampled_poses = statistics.sampled_poses+numel(sample_times);
        statistics.swept_segments = statistics.swept_segments+1;
        statistics.max_sweep_padding_m = max(statistics.max_sweep_padding_m,padding);
    end
end
if strcmp(map.config.trajectory_end_policy,'hold_until_t_max')
    if abs(states(end,4)) > 1e-9
        error('STOccupancy:InvalidTrajectory','只有末速度为零的停车轨迹可延长终点占用。');
    end
    [index,~] = st_resource_index(map,[states(end,1:2),times(end)]);
    cells = st_vehicle_cells(map,states(end,1:3),footprint,0);
    for it = index(3):map.nt
        bins{it} = [bins{it};cells];
    end
end
for it = 1:map.nt
    bins{it} = unique(bins{it})+(it-1)*map.nx*map.ny;
end
[map, write_stats] = st_write_occupancy(map,vertcat(bins{:}),owner_id,3);
statistics.marked_resource_blocks = write_stats.marked_resource_blocks;
statistics.conflict_blocks = write_stats.conflict_blocks;
statistics.new_blocks = write_stats.new_blocks;
map.trajectory_records(end+1) = struct('owner_id',char(owner_id), ...
    'states',states,'stats',statistics);
end

function states = trajectoryStates(trajectory)
%TRAJECTORYSTATES 严格检查状态，不排序、不删除非法点，防止掩盖时间错误。
if isstruct(trajectory) && isscalar(trajectory)
    required = {'x','y','theta','v','t'};
    if ~all(isfield(trajectory,required)) || ...
            isfield(trajectory,'valid') && ~trajectory.valid
        error('STOccupancy:InvalidTrajectory','路径需要x/y/theta/v/t，且不能标记为规划失败。');
    end
    vectors = cell(1,5);
    for k = 1:5
        value = trajectory.(required{k});
        if ~isnumeric(value) || ~isreal(value) || ~isvector(value)
            error('STOccupancy:InvalidTrajectory','轨迹状态字段必须是实数向量。');
        end
        vectors{k} = double(value(:));
    end
    sizes = cellfun(@numel,vectors);
    if any(sizes ~= sizes(1))
        error('STOccupancy:InvalidTrajectory','各状态字段长度必须一致。');
    end
    states = horzcat(vectors{:});
elseif isnumeric(trajectory) && isreal(trajectory) && ismatrix(trajectory) && size(trajectory,2) == 5
    states = double(trajectory);
else
    error('STOccupancy:InvalidTrajectory','轨迹必须为path结构或Nx5状态数组。');
end
if isempty(states) || any(~isfinite(states(:))) || any(diff(states(:,5)) <= 0) || any(states(:,4) < 0)
    error('STOccupancy:InvalidTrajectory','状态需有限、速度非负、时间严格递增；单点允许。');
end
end

function state = interpolatedState(a,b,time)
%INTERPOLATEDSTATE 对短角差yaw与x/y/v做线性时间插值，时间不外推。
fraction = (time-a(5))/(b(5)-a(5));
state = a+(b-a)*fraction;
state(5) = time;
end
