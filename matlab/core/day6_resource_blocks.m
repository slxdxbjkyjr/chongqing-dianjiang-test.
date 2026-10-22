function [indices, outside, resources_checked] = day6_resource_blocks(states, map, footprint, stop_on_hit)
%DAY6_RESOURCE_BLOCKS 栅格化候选车身的层内扫掠，查询动态资源前使用。
% 输入：密集Nx5候选轨迹、Day5图、膨胀车身。输出：索引、越界及实际检查块数。
% 采用与Day5相同的半开时间切段、整块矩形SAT和采样间隙补偿。
% 搜索可在首个动态块命中后退出；最终验收仍收集全部块，不改变拒绝判据。
if nargin < 4, stop_on_hit = false; end
[state_indices,inside] = st_resource_index(map,states(:,[1,2,5]));
outside = any(~inside);
indices = zeros(0,1);
resources_checked = 0;
if outside, return; end
if stop_on_hit && ~isfield(map,'dynamic_layer_cells'), map = prepare_dynamic_context(map,[]); end
% 先生成与原实现完全相同的端点/扫掠矩形，再按层批量检测。
% 搜索模式只查询动态块，最终验收仍返回完整车身资源集。
poses = cell(size(states,1),1); layers = cell(size(states,1),1); paddings = cell(size(states,1),1);
poses{1} = states(:,1:3); layers{1} = state_indices(:,3); paddings{1} = zeros(size(states,1),1);
group = 1;
if ~stop_on_hit, bins = cell(map.nt,1); end
for k = 1:size(states,1)
    if ~stop_on_hit
        cells = st_vehicle_cells(map,states(k,1:3),footprint,0);
        layer = state_indices(k,3);
        bins{layer} = [bins{layer};cells];
    end
end
for k = 1:size(states,1)-1
    a = states(k,:); b = states(k+1,:);
    cuts = [a(5),map.t_edges(map.t_edges > a(5) & map.t_edges < b(5)),b(5)];
    for j = 1:numel(cuts)-1
        first = a+(b-a)*(cuts(j)-a(5))/(b(5)-a(5));
        last = a+(b-a)*(cuts(j+1)-a(5))/(b(5)-a(5));
        duration = last(5)-first(5);
        % 对带运动学控制的弧段，弧长不小于首末点直线距离。
        distance = max(hypot(last(1)-first(1),last(2)-first(2)), ...
            0.5*(first(4)+last(4))*duration);
        bound = distance+footprint.radius*abs(last(3)-first(3));
        count = max([1,ceil(duration/map.config.sweep_max_dt_s), ...
            ceil(bound/map.config.sweep_max_distance_m)]);
        padding = bound/(2*count);
        [index,~] = st_resource_index(map,[first(1:2),(first(5)+last(5))/2]);
        if stop_on_hit
            group = group+1;
            poses{group} = first(1:3)+(last(1:3)-first(1:3)).*((0:count)'/count);
            layers{group} = repmat(index(3),count+1,1);
            paddings{group} = repmat(padding,count+1,1);
        else
            hits = cell(count+1,1);
            for n = 0:count
                state = first+(last-first)*n/count;
                hits{n+1} = st_vehicle_cells(map,state(1:3),footprint,padding);
            end
            bins{index(3)} = [bins{index(3)};vertcat(hits{:})];
        end
    end
end
if stop_on_hit
    [indices,resources_checked] = sparseRectangleHits( ...
        vertcat(poses{:}),vertcat(layers{:}),vertcat(paddings{:}),map,footprint);
    return;
end
% 空时间层无需调用unique；保持原来层号/线性索引的返回次序。
for k = reshape(find(~cellfun('isempty',bins)),1,[])
    bins{k} = unique(bins{k})+(k-1)*map.nx*map.ny;
end
indices = vertcat(bins{:});
resources_checked = numel(indices);
end

function [indices,resources_checked] = sparseRectangleHits(poses,layers,paddings,map,footprint)
%SPARSERECTANGLEHITS 批量四轴SAT：矩形与已占用整块接触也判冲突。
% 纵轴、横轴、世界x/y四个不等式与st_vehicle_cells完全一致；不降低采样密度。
% 每批至多64个姿态，避免长轨迹产生大型临时数组；保留首个命中姿态的顺序。
indices = zeros(0,1); first_hit = inf; resources_checked = 0;
for layer = reshape(unique(layers),1,[])
    cells = map.dynamic_layer_cells{layer}; centers = map.dynamic_layer_centers{layer};
    if isempty(cells), continue; end
    samples = find(layers == layer);
    evaluated_layer = false;
    for begin = 1:64:numel(samples)
        batch = samples(begin:min(begin+63,numel(samples)));
        batch = batch(batch < first_hit);
        if isempty(batch), break; end
        if ~evaluated_layer
            resources_checked = resources_checked+numel(cells); evaluated_layer = true;
        end
        c = cos(poses(batch,3)).'; s = sin(poses(batch,3)).';
        hl = footprint.length/2+paddings(batch).'; hw = footprint.half_width+paddings(batch).';
        rx = centers(:,1)-(poses(batch,1).'+footprint.center_offset*c);
        ry = centers(:,2)-(poses(batch,2).'+footprint.center_offset*s);
        hit = abs(rx) <= hl.*abs(c)+hw.*abs(s)+map.dx/2+1e-12 & ...
            abs(ry) <= hl.*abs(s)+hw.*abs(c)+map.dy/2+1e-12 & ...
            abs(rx.*c+ry.*s) <= hl+map.dx/2*abs(c)+map.dy/2*abs(s)+1e-12 & ...
            abs(-rx.*s+ry.*c) <= hw+map.dx/2*abs(s)+map.dy/2*abs(c)+1e-12;
        column = find(any(hit,1),1);
        if ~isempty(column)
            first_hit = batch(column);
            indices = cells(hit(:,column))+(layer-1)*map.nx*map.ny;
            break;
        end
    end
end
end
