function [collision, detail] = check_vehicle_vehicle_collision(ego, other, ego_vehicle, other_vehicle, safety)
%CHECK_VEHICLE_VEHICLE_COLLISION 按膨胀旋转矩形作最终车辆碰撞判定。
% 输入：两车后轴位姿(至少x/y/theta)、各自车辆配置、Day6安全配置。
% 输出：碰撞、真实车身净距、膨胀车身净距和圆/AABB/SAT诊断。
% 圆与AABB只作快速筛选；SAT四轴处理分离，净距为多边形边界最短距离。
if numel(ego) < 3 || numel(other) < 3 || any(~isfinite([ego(1:3),other(1:3)]))
    error('DynamicCollision:InvalidTrajectory','碰撞位姿需要有限x/y/theta。');
end
extra = safety.minimum_safety_distance_m/2;
if isfield(safety,'sweep_padding_m'), extra = extra+safety.sweep_padding_m; end
raw_ego = inflate_vehicle_occupancy(withoutMargin(ego_vehicle),0);
raw_other = inflate_vehicle_occupancy(withoutMargin(other_vehicle),0);
inflated_ego = inflate_vehicle_occupancy(ego_vehicle,extra);
inflated_other = inflate_vehicle_occupancy(other_vehicle,extra);
a = globalCorners(ego,inflated_ego); b = globalCorners(other,inflated_other);
circle = hypot(ego(1)-other(1),ego(2)-other(2)) <= inflated_ego.radius+inflated_other.radius;
aabb = circle && all(max(a,[],1) >= min(b,[],1)) && all(max(b,[],1) >= min(a,[],1));
rectangle = aabb && rectanglesOverlap(a,b);
collision = rectangle;
raw_a = globalCorners(ego,raw_ego); raw_b = globalCorners(other,raw_other);
raw_distance = polygonDistance(raw_a,raw_b);
detail = struct('minimum_distance_m',raw_distance,'raw_rectangle_overlap',raw_distance==0, ...
    'inflated_distance_m',polygonDistance(a,b),'circle_overlap',circle, ...
    'aabb_overlap',aabb,'rectangle_overlap',rectangle,'ego_corners',a,'other_corners',b);
end

function vehicle = withoutMargin(vehicle)
%WITHOUTMARGIN 真车身净距不含人为碰撞裕度。
vehicle.collision_margin_m = struct('front',0,'rear',0,'side',0);
end

function corners = globalCorners(pose,footprint)
%GLOBALCORNERS 后轴局部矩形旋转和平移到全局坐标。
rotation = [cos(pose(3)),-sin(pose(3));sin(pose(3)),cos(pose(3))];
corners = footprint.corners_local*rotation.'+reshape(pose(1:2),1,2);
end

function overlap = rectanglesOverlap(a,b)
%RECTANGLESOVERLAP 使用两个矩形的纵/横方向共四个分离轴。
edges = [a(2,:)-a(1,:);a(3,:)-a(2,:);b(2,:)-b(1,:);b(3,:)-b(2,:)];
axes = [-edges(:,2),edges(:,1)];
overlap = true;
for k = 1:4
    pa = a*axes(k,:).'; pb = b*axes(k,:).';
    if max(pa) < min(pb)-1e-12 || max(pb) < min(pa)-1e-12
        overlap = false; return;
    end
end
end

function distance = polygonDistance(a,b)
%POLYGONDISTANCE 相交时零，否则取两矩形顶点到对方所有边的最短距离。
if rectanglesOverlap(a,b), distance = 0; return; end
distance = min([vertexEdgeDistance(a,b),vertexEdgeDistance(b,a)]);
end

function distances = vertexEdgeDistance(vertices,polygon)
%VERTEXEDGEDISTANCE 线段投影包含垂足和端点，不用角点间距替代车身净距。
distances = inf(1,4);
for k = 1:4
    a = polygon(k,:); b = polygon(mod(k,4)+1,:); edge = b-a;
    fraction = sum((vertices-a).*edge,2)/sum(edge.^2);
    fraction = max(0,min(1,fraction));
    displacement = vertices-a-fraction.*edge;
    distances(k) = min(hypot(displacement(:,1),displacement(:,2)));
end
end
