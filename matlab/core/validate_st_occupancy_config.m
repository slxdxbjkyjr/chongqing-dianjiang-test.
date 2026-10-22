function [cfg, grid_size] = validate_st_occupancy_config(planner_config)
%VALIDATE_ST_OCCUPANCY_CONFIG 校验资源块配置与Day4搜索时间步兼容性。
% 输入：集中planner_config结构。输出：st_occupancy配置及[nx,ny,nt]。
% 逻辑：资源块均为半开区间；跨度必须整除分辨率；搜索步为dt的整数倍。
if ~isstruct(planner_config) || ~isscalar(planner_config) || ...
        ~isfield(planner_config, 'st_occupancy') || ~isfield(planner_config, 'vhybrid')
    error('STOccupancy:InvalidConfig', '需要完整planner_config及st_occupancy/vhybrid配置。');
end
cfg = planner_config.st_occupancy;
if ~isstruct(cfg) || ~isscalar(cfg) || ~isstruct(planner_config.vhybrid) || ...
        ~isscalar(planner_config.vhybrid)
    error('STOccupancy:InvalidConfig', 'st_occupancy和vhybrid必须为标量结构。');
end
required = {'x_min','x_max','y_min','y_max','t_min','t_max','dx','dy','dt', ...
    'safety_distance_m','sweep_max_dt_s','sweep_max_distance_m'};
for k = 1:numel(required)
    name = required{k};
    if ~isfield(cfg,name) || ~isnumeric(cfg.(name)) || ~isscalar(cfg.(name)) || ...
            ~isreal(cfg.(name)) || ~isfinite(cfg.(name))
        error('STOccupancy:InvalidConfig', '配置字段%s必须是有限实数标量。',name);
    end
end
if ~isfield(cfg,'coordinate_convention') || ~strcmp(cfg.coordinate_convention,'half_open') || ...
        ~isfield(cfg,'trajectory_end_policy') || ...
        ~any(strcmp(cfg.trajectory_end_policy,{'release','hold_until_t_max'}))
    error('STOccupancy:InvalidConfig', '坐标必须为half_open；终点策略必须显式配置。');
end
span = [cfg.x_max-cfg.x_min,cfg.y_max-cfg.y_min,cfg.t_max-cfg.t_min];
resolution = [cfg.dx,cfg.dy,cfg.dt];
if any(span <= 0) || any(resolution <= 0) || cfg.safety_distance_m < 0 || ...
        cfg.sweep_max_dt_s <= 0 || cfg.sweep_max_dt_s > cfg.dt || cfg.sweep_max_distance_m <= 0
    error('STOccupancy:InvalidConfig', '范围、分辨率、安全裕度或扫掠采样参数无效。');
end
ratio = span ./ resolution;
if any(abs(ratio-round(ratio)) > 1e-9) || any(round(ratio) < 1)
    error('STOccupancy:InvalidConfig', '空间和时间跨度必须为相应分辨率的整数倍。');
end
grid_size = round(ratio);
if ~isfield(planner_config.vhybrid,'time_step_s')
    error('STOccupancy:TimeStepMismatch', '缺少vhybrid.time_step_s。');
end
step = planner_config.vhybrid.time_step_s;
if ~isnumeric(step) || ~isscalar(step) || ~isreal(step) || ~isfinite(step) || step <= 0
    error('STOccupancy:TimeStepMismatch', '搜索time_step_s必须是正有限实数。');
end
ratio = step / cfg.dt;
if ratio < 1 || abs(ratio-round(ratio)) > 1e-9
    error('STOccupancy:TimeStepMismatch', '搜索time_step_s必须等于dt或为dt的整数倍。');
end
limits = {'max_resource_blocks','max_sweep_samples_per_segment'};
for k = 1:numel(limits)
    if isfield(cfg,limits{k})
        limit = cfg.(limits{k});
        if ~isnumeric(limit) || ~isscalar(limit) || ~isreal(limit) || ~isfinite(limit) || ...
                limit < 1 || limit ~= round(limit)
            error('STOccupancy:InvalidConfig', '资源和采样上限必须为正有限整数。');
        end
    end
end
if isfield(cfg,'max_resource_blocks') && prod(grid_size) > cfg.max_resource_blocks
    error('STOccupancy:InvalidConfig', '资源块数量超过配置的内存保护上限。');
end
end
