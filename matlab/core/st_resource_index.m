function [indices, inside] = st_resource_index(map, coordinates)
%ST_RESOURCE_INDEX 标记/查询共用的半开区间索引转换。
% 输入：[x,y,t]的N×3有限坐标。输出：N×3索引及每行是否在范围内。
% 极小浮点舍入在整数层边界吸附；上界仍先按严格半开范围排除。
if ~isnumeric(coordinates) || size(coordinates,2) ~= 3 || ~isreal(coordinates)
    error('STOccupancy:InvalidQuery','坐标必须是实数N×3数组。');
end
lower = [map.x_min,map.y_min,map.t_min];
upper = [map.x_max,map.y_max,map.t_max];
scaled = (double(coordinates)-lower)./[map.dx,map.dy,map.dt];
rounded = round(scaled);
snap = abs(scaled-rounded) <= 16*eps(max(1,abs(scaled)));
scaled(snap) = rounded(snap);
indices = floor(scaled)+1;
inside = all(isfinite(coordinates) & coordinates >= lower & coordinates < upper,2);
% 合法坐标略小于最大边界时，吸附不能生成nx+1/ny+1/nt+1。
indices(inside,:) = min(max(indices(inside,:),1),[map.nx,map.ny,map.nt]);
indices(~isfinite(coordinates)) = NaN;
end
