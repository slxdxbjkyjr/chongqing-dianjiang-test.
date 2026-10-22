function [is_valid, normalized, error_code, message] = validate_plan_request(request, runtime)
%VALIDATE_PLAN_REQUEST 检查对外规划请求并补齐默认状态字段。
% 输入：
%   request - 工作人员或MATLAB Function Block提供的请求结构体。
%   runtime - load_summon_runtime_config返回的默认配置。
% 输出：
%   is_valid  - 逻辑标志。
%   normalized - 归一化后的请求；start/goal_state均为[x y yaw v t]。
%   error_code - 100表示输入不合法，101表示配置不合法。
%   message - 面向调用方的中文错误说明。
% 约定：x/y单位为m，yaw为rad，v为非负m/s，t单位为s。

is_valid = false;
% 先建立完整的安全停车默认值。这样即使配置读取或校验过程抛出
% 异常，调用方也能得到确定长度的零速度输出，而不是空结构体。
normalized = struct('start_state',zeros(1,5));
error_code = uint8(100);

if nargin < 2 || ~isstruct(runtime) || ~isscalar(runtime)
    error_code = uint8(101);
    message = '运行时配置结构体缺失。';
    return;
end
if ~isstruct(request) || ~isscalar(request)
    message = 'request必须是标量结构体。';
    return;
end

try
    normalized = request;
    normalized.vehicle_config = chooseField(request,'vehicle_config',runtime.vehicle_config);
    normalized.planner_config = chooseField(request,'planner_config',runtime.planner_config);

    if isfield(request,'map')
        normalized.map = request.map;
    elseif isfield(request,'static_map')
        normalized.map = request.static_map;
    else
        normalized.map = [];
    end

    if ~isfield(request,'start_state')
        message = '缺少request.start_state。';
        return;
    end
    if ~isfield(request,'goal_state')
        message = '缺少request.goal_state。';
        return;
    end
    [normalized.start_state, ok, msg] = normalizeState(request.start_state,'start_state');
    if ~ok, message = msg; return; end
    [normalized.goal_state, ok, msg] = normalizeState(request.goal_state,'goal_state');
    if ~ok, message = msg; return; end

    % 对外输出时间轴固定从0开始；非零起始时间会导致搜索时间、动态
    % 障碍物时间层和固定采样输出含义不一致，因此接口明确拒绝该请求。
    if abs(normalized.start_state(5)) > 1e-12 || abs(normalized.goal_state(5)) > 1e-12
        message = 'start_state和goal_state的时间必须为0；输出轨迹从t=0开始。';
        return;
    end
    if normalized.start_state(4) < 0 || normalized.goal_state(4) < 0
        message = '速度v必须是非负值；倒车方向通过gear单独表示。';
        return;
    end
    if ~isfield(normalized.planner_config,'dynamics') || ...
            ~isfield(normalized.planner_config,'vhybrid')
        error_code = uint8(101);
        message = 'planner_config缺少dynamics或vhybrid。';
        return;
    end
    dynamics = normalized.planner_config.dynamics;
    if normalized.start_state(4) > dynamics.v_max_mps + 1e-12 || ...
            normalized.goal_state(4) > dynamics.v_max_mps + 1e-12
        message = '起点或目标速度超过配置的最大速度。';
        return;
    end
    if isfield(normalized.planner_config.vhybrid,'output_dt_s')
        output_dt = normalized.planner_config.vhybrid.output_dt_s;
    else
        output_dt = 0.1;
    end
    if ~isnumeric(output_dt) || ~isscalar(output_dt) || ~isfinite(output_dt) || output_dt <= 0
        error_code = uint8(101);
        message = '输出采样周期必须是正有限数。';
        return;
    end
    normalized.output_dt_s = output_dt;
    if isfield(request,'output_dt_s')
        normalized.output_dt_s = request.output_dt_s;
    end
    if ~isnumeric(normalized.output_dt_s) || ~isscalar(normalized.output_dt_s) || ...
            ~isfinite(normalized.output_dt_s) || normalized.output_dt_s <= 0
        message = 'request.output_dt_s必须是正有限数。';
        return;
    end

    normalized.max_output_time_s = 60.0;
    if isfield(request,'max_output_time_s')
        normalized.max_output_time_s = request.max_output_time_s;
    end
    if ~isnumeric(normalized.max_output_time_s) || ~isscalar(normalized.max_output_time_s) || ...
            ~isfinite(normalized.max_output_time_s) || normalized.max_output_time_s <= 0
        message = 'request.max_output_time_s必须是正有限数。';
        return;
    end
    ratio = normalized.max_output_time_s/normalized.output_dt_s;
    if abs(ratio-round(ratio)) > 1e-9
        message = '最大输出时域必须是采样周期的整数倍。';
        return;
    end

    if ~isfield(normalized,'options') || isempty(normalized.options)
        normalized.options = struct();
    elseif ~isstruct(normalized.options) || ~isscalar(normalized.options)
        message = 'request.options必须是标量结构体。';
        return;
    end
    if isfield(request,'higher_priority_trajectories')
        normalized.higher_priority_trajectories = request.higher_priority_trajectories;
    elseif isfield(request,'dynamic_obstacles')
        normalized.higher_priority_trajectories = request.dynamic_obstacles;
    else
        normalized.higher_priority_trajectories = [];
    end

    % 只有在调用方提供已经构造好的地图时才在此处检查其结构；
    % map_config由主接口根据默认配置或请求覆盖项构造。
    if ~isempty(normalized.map) && (~isstruct(normalized.map) || ...
            ~isfield(normalized.map,'boundary_xy') || ~isfield(normalized.map,'obstacles'))
        message = 'request.map必须是summon_map生成的地图结构。';
        return;
    end
    is_valid = true;
    error_code = uint8(0);
    message = '输入检查通过。';
catch exception
    is_valid = false;
    error_code = uint8(101);
    message = ['配置检查失败：',exception.message];
end
end

function value = chooseField(request, field_name, default_value)
if isfield(request,field_name) && ~isempty(request.(field_name))
    value = request.(field_name);
else
    value = default_value;
end
end

function [state, is_valid, message] = normalizeState(value, field_name)
state = zeros(1,5);
is_valid = false;
message = '';
if ~isnumeric(value) || ~isreal(value) || ~isvector(value) || ...
        ~ismember(numel(value),[3,4,5]) || any(~isfinite(value(:)))
    message = [field_name,'必须是有限实数[x y yaw]、[x y yaw v]或[x y yaw v t]。'];
    return;
end
value = double(value(:).');
state(1:numel(value)) = value;
if numel(value) == 3, state(4:5) = [0,0]; end
if numel(value) == 4, state(5) = 0; end
is_valid = true;
end
