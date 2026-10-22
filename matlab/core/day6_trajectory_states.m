function states = day6_trajectory_states(trajectory)
%DAY6_TRAJECTORY_STATES 检查公共轨迹接口并解开航向角表示环绕。
% 输入：有限Nx5或x/y/theta/v/t结构；输出：时间严格递增、后轴参考状态数组。
% v 是非负速度大小，倒车方向单独保存于可选 gear=+1/-1 字段；
% gear 不加入 Nx5 数组，避免与原 Day3/Day5 接口的速度约定混淆。
if isstruct(trajectory) && isscalar(trajectory)
    fields = {'x','y','theta','v','t'};
    if ~all(isfield(trajectory,fields)) || isfield(trajectory,'valid') && ~trajectory.valid
        error('DynamicCollision:InvalidTrajectory','轨迹需要x/y/theta/v/t且必须有效。');
    end
    parts = cell(1,5);
    for k = 1:5
        value = trajectory.(fields{k});
        if ~isnumeric(value) || ~isreal(value) || ~isvector(value)
            error('DynamicCollision:InvalidTrajectory','轨迹状态字段必须为实数向量。');
        end
        parts{k} = double(value(:));
    end
    sizes = cellfun(@numel,parts);
    if any(sizes ~= sizes(1))
        error('DynamicCollision:InvalidTrajectory','状态字段长度必须一致。');
    end
    states = horzcat(parts{:});
    if isfield(trajectory,'gear')
        gears = trajectory.gear;
        if ~isnumeric(gears) || ~isreal(gears) || ~isvector(gears) || ...
                numel(gears) ~= size(states,1) || any(gears(:) ~= 1 & gears(:) ~= -1)
            error('DynamicCollision:InvalidTrajectory','gear 必须是与状态等长的 +1/-1 档位向量。');
        end
    end
elseif isnumeric(trajectory) && isreal(trajectory) && ismatrix(trajectory) && size(trajectory,2) == 5
    states = double(trajectory);
else
    error('DynamicCollision:InvalidTrajectory','需要path结构或Nx5状态。');
end
if isempty(states) || any(~isfinite(states(:))) || any(diff(states(:,5)) <= 0) || any(states(:,4) < 0)
    error('DynamicCollision:InvalidTrajectory','状态需有限、速度非负、时间严格递增。');
end
states(:,3) = unwrap(states(:,3));
end
