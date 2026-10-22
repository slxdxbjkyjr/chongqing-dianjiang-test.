function map = summon_map(map_config)
%SUMMON_MAP 将地图 JSON 结构转换为独立召集地图结构。
%
% 输入：
%   map_config - map_config.json 经 jsondecode 得到的结构体。
%
% 输出：
%   map.boundary_xy - 闭合道路边界顶点 N×2。
%   map.centerline_xy - 可选道路中心线。
%   map.obstacles - 静态障碍物多边形 cell 数组。
%   map.bounds - [min_x, min_y, max_x, max_y] 搜索范围。
%
% 算法逻辑：
%   只保留召集场景需要的几何地图，不携带 APA 车位、超声波或自由空间总线。

if nargin < 1 || ~isfield(map_config, 'boundary')
    error('Summon:Map:InvalidConfig', 'map_config 缺少 boundary。');
end

map = struct();
map.boundary_xy = double(map_config.boundary.vertices_xy);
if size(map.boundary_xy, 2) ~= 2 || size(map.boundary_xy, 1) < 4
    error('Summon:Map:InvalidBoundary', 'boundary.vertices_xy 必须是 N×2 多边形。');
end
if any(map.boundary_xy(1,:) ~= map.boundary_xy(end,:))
    map.boundary_xy(end+1,:) = map.boundary_xy(1,:);
end

if isfield(map_config, 'road') && isfield(map_config.road, 'centerline_xy')
    map.centerline_xy = double(map_config.road.centerline_xy);
else
    map.centerline_xy = zeros(0, 2);
end

map.obstacles = cell(0, 1);
if isfield(map_config, 'static_obstacles')
    for k = 1:numel(map_config.static_obstacles)
        map.obstacles{end+1,1} = double(map_config.static_obstacles(k).vertices_xy); %#ok<AGROW>
    end
end

map.bounds = [min(map.boundary_xy(:,1)), min(map.boundary_xy(:,2)), ...
    max(map.boundary_xy(:,1)), max(map.boundary_xy(:,2))];
end
