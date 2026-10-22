function obstacle = dynamic_vehicle_obstacle(trajectory, vehicle_config, owner_id, planner_config)
%DYNAMIC_VEHICLE_OBSTACLE 将高优先级轨迹封装为带车辆几何的动态障碍物。
% 输入：path或Nx5[x,y,theta,v,t]、车辆配置、外部ID、集中规划配置。
% 输出：id/states/vehicle_config/footprint/hold_end；停车驻留由Day6配置决定。
states = day6_trajectory_states(trajectory);
if ~(ischar(owner_id) && isrow(owner_id) || isstring(owner_id) && isscalar(owner_id)) || isempty(char(owner_id))
    error('DynamicCollision:InvalidOwner','车辆ID必须为非空字符串。');
end
hold_end = planner_config.day6.goal_hold_enabled;
if hold_end && abs(states(end,4)) > 1e-9
    error('DynamicCollision:InvalidTrajectory','停车驻留轨迹末速度必须为零。');
end
footprint = inflate_vehicle_occupancy(vehicle_config,planner_config.day6.minimum_safety_distance_m/2);
obstacle = struct('id',char(owner_id),'states',states,'vehicle_config',vehicle_config, ...
    'footprint',footprint,'hold_end',logical(hold_end));
end
