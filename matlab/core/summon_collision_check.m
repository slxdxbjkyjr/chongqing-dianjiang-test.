function [is_collision, detail] = summon_collision_check(pose, map, vehicle_config)
%SUMMON_COLLISION_CHECK 检查车辆外包络与道路边界/静态障碍物碰撞。
%
% 输入：
%   pose          - [x,y,theta] 当前车辆位姿。
%   map           - summon_map 生成的地图结构。
%   vehicle_config- 显式车辆尺寸和安全裕度配置。
%
% 输出：
%   is_collision - logical 标志；true 表示碰撞或越界。
%   detail       - 包含碰撞类型、车辆角点和障碍物索引。
%
% 算法逻辑：
%   车辆采用带安全裕度的旋转矩形。先检查四个角点是否在道路边界内，
%   再检查角点是否落入静态障碍物。该接口不调用 APA 的车位碰撞状态机。

if numel(pose) ~= 3
    error('Summon:Collision:InvalidPose', 'pose 必须为 [x,y,theta]。');
end

margin = vehicle_config.collision_margin_m;
half_width = (double(vehicle_config.dimensions_m.width) + 2 * double(margin.side)) / 2;
front = double(vehicle_config.dimensions_m.front_axle_to_front) + ...
    double(vehicle_config.dimensions_m.wheelbase) + double(margin.front);
rear = double(vehicle_config.dimensions_m.rear_axle_to_rear) + double(margin.rear);

local_corners = [front, half_width; front, -half_width; ...
    -rear, -half_width; -rear, half_width];
rotation = [cos(pose(3)), -sin(pose(3)); sin(pose(3)), cos(pose(3))];  
corners = (rotation * local_corners.').';
corners = corners + pose(1:2);

inside_boundary = inpolygon(corners(:,1), corners(:,2), ...
    map.boundary_xy(:,1), map.boundary_xy(:,2));
is_collision = ~all(inside_boundary);
collision_type = 'none';
obstacle_index = 0;

if is_collision
    collision_type = 'road_boundary';
else
    for k = 1:numel(map.obstacles)
        obstacle = map.obstacles{k};
        if any(inpolygon(corners(:,1), corners(:,2), obstacle(:,1), obstacle(:,2)))
            is_collision = true;
            collision_type = 'static_obstacle';
            obstacle_index = k;
            break;
        end
    end
end

detail = struct('type', collision_type, 'obstacle_index', obstacle_index, ...
    'corners_xy', corners);
end
