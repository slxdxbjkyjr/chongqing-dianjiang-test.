function [is_occupied, owner_type, owner_id, indices, detail] = query_occupancy(x, y, t, map)
%QUERY_OCCUPANCY 查询半开资源块，返回占用/类型/外部ID/[ix,iy,it]及详情。
% 输入：标量x,y,t与时空地图。输出：越界恒为不可行；重叠块detail保留全部ID。
% owner_id为第一占用者外部ID；detail.owner_ids/types/reasons提供全部归属。
if ~isnumeric(x) || ~isnumeric(y) || ~isnumeric(t) || ...
        ~isscalar(x) || ~isscalar(y) || ~isscalar(t) || ~isreal([x,y,t])
    error('STOccupancy:InvalidQuery','查询坐标必须为实数标量。');
end
[indices,inside] = st_resource_index(map,[x,y,t]);
detail = struct('reason','free','owner_ids',{{}},'owner_types',{{}}, ...
    'reasons',{{}},'permanent',false,'block_bounds',nan(3,2));
if ~inside
    is_occupied = true; owner_type = 'out_of_bounds'; owner_id = '';
    detail.reason = 'outside_time_space_bounds'; detail.permanent = true;
    return;
end
detail.block_bounds = [map.x_edges(indices(1):indices(1)+1); ...
    map.y_edges(indices(2):indices(2)+1); map.t_edges(indices(3):indices(3)+1)];
index = sub2ind([map.nx,map.ny,map.nt],indices(1),indices(2),indices(3));
is_occupied = map.occupancy(index);
owner_type = map.type_labels{double(map.owner_type(index))+1};
detail.reason = map.reason_labels{double(map.reason(index))+1};
owner_id = '';
if ~is_occupied, return; end
ids = map.owner_id(index);
entry = find([map.overlaps.linear_index] == index,1);
if ~isempty(entry), ids = map.overlaps(entry).owner_ids; end
owner_id = map.owners(map.owner_id(index)).id;
detail.owner_ids = {map.owners(ids).id};
detail.owner_types = map.type_labels(double([map.owners(ids).type])+1);
detail.reasons = {map.owners(ids).reason};
detail.permanent = any([map.owners(ids).type] <= 2);
end
