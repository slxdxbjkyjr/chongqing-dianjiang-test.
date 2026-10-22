function [map, stats] = st_write_occupancy(map, linear_indices, owner_id, owner_type)
%ST_WRITE_OCCUPANCY 写入资源块并保留全部重叠归属（值结构，不含handle字典）。
% 输入：三维数组线性索引、外部占用者ID、类型1边界/2静态/3动态。
% 输出：新地图与新增/冲突块统计；稠密owner_id保留第一占用者索引。
if ~(ischar(owner_id) && isrow(owner_id) || isstring(owner_id) && isscalar(owner_id)) || ...
        isempty(char(owner_id)) || ~ismember(owner_type,[1,2,3])
    error('STOccupancy:InvalidOwner','占用者ID必须非空，类型必须为1/2/3。');
end
owner_id = char(owner_id);
registered = find(strcmp({map.owners.id},owner_id),1);
if isempty(registered)
    registered = numel(map.owners)+1;
    map.owners(registered) = struct('id',owner_id,'type',uint8(owner_type), ...
        'reason',map.reason_labels{owner_type+1});
elseif map.owners(registered).type ~= owner_type
    error('STOccupancy:InvalidOwner','相同ID不能用于不同占用类型。');
end
indices = unique(double(linear_indices(:)));
% 写入后旧动态掩码/包络/稀疏块缓存失效；下次查询须从新归属重建。
cache_fields = intersect(fieldnames(map),{'dynamic_occupancy','dynamic_owner_index', ...
    'dynamic_layer_bounds','dynamic_layer_cells','dynamic_layer_centers','dynamic_context_ids'});
if ~isempty(cache_fields), map = rmfield(map,cache_fields); end
old = map.owner_id(indices);
new_indices = indices(old == 0);
other_indices = indices(old ~= 0 & old ~= registered);
map.occupancy(indices) = true;
map.owner_id(new_indices) = uint32(registered);
map.owner_type(new_indices) = uint8(owner_type);
map.reason(new_indices) = uint8(owner_type);
% 仅重叠块需要多归属列表；重复写同一占用者不产生额外冲突。
[existing,entries] = ismember(other_indices,[map.overlaps.linear_index]);
fresh = other_indices(~existing);
added = repmat(struct('linear_index',0,'owner_ids',uint32([])),1,numel(fresh));
for k = 1:numel(fresh)
    added(k).linear_index = fresh(k);
    added(k).owner_ids = uint32([map.owner_id(fresh(k)),registered]);
end
map.overlaps = [map.overlaps,added];
entries = entries(existing);
for k = 1:numel(entries)
    entry = entries(k);
    map.overlaps(entry).owner_ids = unique([map.overlaps(entry).owner_ids,uint32(registered)],'stable');
end
map.owner_type(other_indices) = uint8(4);
map.reason(other_indices) = uint8(4);
stats = struct('new_blocks',numel(new_indices),'conflict_blocks',numel(other_indices), ...
    'marked_resource_blocks',numel(indices));
end
