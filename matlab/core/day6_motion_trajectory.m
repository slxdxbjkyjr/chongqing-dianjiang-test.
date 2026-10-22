function trajectory = day6_motion_trajectory(start, next, steering, acceleration, vehicle, planner, gear, model_samples)
%DAY6_MOTION_TRAJECTORY 对搜索边用同一个Day3运动学模型生成中间状态。
% 输入：首末[x,y,theta,v,t]、转角(rad)、速度大小的加速度(m/s^2)、
% 车辆和配置，以及可选档位gear=+1/-1(默认前进)。输出：带控制量和档位的密集path。
% 时间与弧长约束共同限制采样；包括起点、终点及减速边中点。
% 可选 model_samples 为同一首状态/控制/档位经模型直接积分且已验证的 Nx5
% 内部缓存。只复用原要求采样时刻的模型状态；未命中时重新积分，禁止插值。
if nargin < 7, gear = 1; end
if nargin < 8, model_samples = []; end
duration = next(5)-start(5);
if duration <= 0
    error('DynamicCollision:InvalidTrajectory','搜索边时间必须严格增加。');
end
distance = 0.5*(abs(start(4))+abs(next(4)))*duration;
count = max([2,ceil(duration/planner.day6.collision_sample_step_s), ...
    ceil(distance/planner.day6.collision_sample_step_m)]);
sample_times = unique([linspace(0,duration,count+1),duration/2]);
states = zeros(numel(sample_times),5); states(1,:) = start;
locations = zeros(numel(sample_times),1);
if ~isempty(model_samples)
    if ~isnumeric(model_samples) || ~isreal(model_samples) || ~ismatrix(model_samples) || size(model_samples,2) ~= 5 || ...
            any(~isfinite(model_samples(:))) || any(model_samples(:,4) < 0) || ...
            any(diff(model_samples(:,5)) <= 0)
        error('DynamicCollision:InvalidTrajectory','模型缓存须为有限、速度非负且时间递增的 Nx5 状态。');
    end
    % 缓存不能绕过原有档位/转角/加速度/速度约束；完整动作仍由原模型校验。
    [~,~,~,valid] = summon_vehicle_dynamic_gear(start,steering,acceleration,duration,vehicle,planner,gear);
    if ~valid
        error('DynamicCollision:InvalidTrajectory','搜索边违反运动学约束。');
    end
    required_times = start(5)+sample_times;
    [differences,matched] = min(abs(model_samples(:,5)-required_times),[],1);
    % 积分累计绝对时间可能产生机器舍入差；只容忍这种等时刻表示误差。
    matched(differences > 64*eps(max(1,max(abs(required_times))))) = 0;
    locations = matched(:);
end
for k = 2:numel(sample_times)
    if locations(k) > 0
        states(k,:) = model_samples(locations(k),:);
        states(k,5) = start(5)+sample_times(k);
        continue;
    end
    [states(k,:),~,~,valid] = summon_vehicle_dynamic_gear( ...
        start,steering,acceleration,sample_times(k),vehicle,planner,gear);
    if ~valid
        error('DynamicCollision:InvalidTrajectory','搜索边违反运动学约束。');
    end
end
trajectory = struct('x',states(:,1),'y',states(:,2),'theta',states(:,3), ...
    'v',states(:,4),'t',states(:,5),'acceleration',[0;repmat(acceleration,numel(sample_times)-1,1)], ...
    'steering_angle',[0;repmat(steering,numel(sample_times)-1,1)], ...
    'gear',repmat(gear,numel(sample_times),1),'valid',true);
end
