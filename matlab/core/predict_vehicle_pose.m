function [states, active] = predict_vehicle_pose(obstacle, time)
%PREDICT_VEHICLE_POSE 按同一绝对时间预测动态车后轴位姿。
% 输入：dynamic_vehicle_obstacle结果、标量或向量时间(s)。输出：Nx5及活跃标志。
% 起点之前不外推；末点后仅配置停车驻留时保持位置/航向与零速度。
if ~isnumeric(time) || ~isreal(time) || any(~isfinite(time(:)))
    error('DynamicCollision:InvalidTime','预测时间必须为有限实数。');
end
time = double(time(:));
data = obstacle.states;
states = nan(numel(time),5);
active = time >= data(1,5) & (time <= data(end,5) | obstacle.hold_end);
for k = 1:numel(time)
    if ~active(k), continue; end
    t = time(k);
    if t >= data(end,5)
        states(k,:) = data(end,:);
        if t > data(end,5), states(k,4) = 0; end
    else
        index = find(data(:,5) <= t,1,'last');
        fraction = (t-data(index,5))/(data(index+1,5)-data(index,5));
        states(k,:) = data(index,:)+(data(index+1,:)-data(index,:))*fraction;
    end
    states(k,3) = mod(states(k,3)+pi,2*pi)-pi;
    states(k,5) = t;
end
end
