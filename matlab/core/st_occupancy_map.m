function map = st_occupancy_map(planner_config, geometry)
%ST_OCCUPANCY_MAP 创建occupancy(ix,iy,it)和占用归属/原因的值结构。
% 输入：完整集中配置，可选summon_map静态几何。输出：空时空资源地图。
% 坐标：块[i]为[edge(i),edge(i+1))；coords是块中心，不是查询取整点。
% owner_id为owners注册表索引；重叠块在overlaps中保存全部注册表索引。
if nargin < 2, geometry = struct(); end
[cfg, dims] = validate_st_occupancy_config(planner_config);
map = struct('config',cfg,'geometry',geometry,'nx',dims(1),'ny',dims(2),'nt',dims(3));
map.x_min = cfg.x_min; map.x_max = cfg.x_max; map.dx = cfg.dx;
map.y_min = cfg.y_min; map.y_max = cfg.y_max; map.dy = cfg.dy;
map.t_min = cfg.t_min; map.t_max = cfg.t_max; map.dt = cfg.dt;
map.x_edges = cfg.x_min+(0:map.nx)*cfg.dx;
map.y_edges = cfg.y_min+(0:map.ny)*cfg.dy;
map.t_edges = cfg.t_min+(0:map.nt)*cfg.dt;
map.x_coords = (map.x_edges(1:end-1)+map.x_edges(2:end))/2;
map.y_coords = (map.y_edges(1:end-1)+map.y_edges(2:end))/2;
map.t_coords = (map.t_edges(1:end-1)+map.t_edges(2:end))/2;
map.occupancy = false(dims);
map.owner_type = zeros(dims,'uint8');
map.owner_id = zeros(dims,'uint32');
map.reason = zeros(dims,'uint8');
map.type_labels = {'free','boundary','static_obstacle','dynamic_vehicle','multiple_owners'};
map.reason_labels = {'free','permanent_boundary_inflation','permanent_static_inflation', ...
    'dynamic_trajectory_sweep','multiple_owners'};
map.owners = struct('id',{},'type',{},'reason',{});
map.overlaps = struct('linear_index',{},'owner_ids',{});
map.trajectory_records = struct('owner_id',{},'states',{},'stats',{});
end
