function [outside,detail] = vhybrid_curve_boundary_check(start,curve,radius,map,vehicle,gear)
%VHYBRID_CURVE_BOUNDARY_CHECK 解析排除必定穿出矩形道路的完整Dubins曲线。
% 输入：后轴起始位姿、三段Dubins几何、最小半径、原始地图和车辆配置。
% 输出：outside为已证明越界；detail记录是否适用解析证明、包络和拒绝段。
% 判据：直线中车角轨迹为线段；圆弧中每个车角绕相同瞬时转动中心
% 转过相同角度。检查弧的两个端点和区间内所有x/y极值，得到完整包络。
% 只对轴对齐矩形道路启用；其他道路返回未证明，仍由逐段碰撞检测验收。
% 该剪枝只拒绝确定不可行的几何，与速度无关，不引导或替代路径搜索。
outside = false;
if nargin < 6, gear = 1; end
detail = struct('evaluated',false,'swept_bounds',nan(1,4),'first_rejected_segment',0);
boundary = map.boundary_xy;
if isequal(boundary(1,:),boundary(end,:)), boundary(end,:) = []; end
if size(boundary,1)~=4, return; end
bounds = [min(boundary(:,1)),min(boundary(:,2)),max(boundary(:,1)),max(boundary(:,2))];
tol = 1e-9;
on_x = abs(boundary(:,1)-bounds(1))<=tol | abs(boundary(:,1)-bounds(3))<=tol;
on_y = abs(boundary(:,2)-bounds(2))<=tol | abs(boundary(:,2)-bounds(4))<=tol;
codes = double(abs(boundary(:,1)-bounds(3))<=tol) + ...
    2*double(abs(boundary(:,2)-bounds(4))<=tol);
if ~all(on_x & on_y) || numel(unique(codes))~=4, return; end
detail.evaluated = true;
footprint = inflate_vehicle_occupancy(vehicle,0);
state = double(start(1:3)); state = state(:).';
% 几何沿运动切向展开；倒车时同时旋转局部车身坐标，保留后轴偏心。
if gear<0, state(3) = state(3)+pi; footprint.corners_local = -footprint.corners_local; end
swept = [Inf,Inf,-Inf,-Inf];
for segment = 1:3
    length_m = curve.lengths(segment); turn = curve.types(segment);
    theta = state(3); c = cos(theta); s = sin(theta);
    corners = footprint.corners_local*[c,s;-s,c]+state(1:2);
    if turn==0
        points = [corners;corners+length_m*[c,s]];
        state(1:2) = state(1:2)+length_m*[c,s];
    else
        signed_radius = radius/turn;
        center = state(1:2)+signed_radius*[-s,c];
        angle_change = length_m/signed_radius;
        points = zeros(0,2);
        for corner = 1:4
            vector = corners(corner,:)-center;
            corner_radius = hypot(vector(1),vector(2));
            begin_angle = atan2(vector(2),vector(1));
            end_angle = begin_angle+angle_change;
            low = min(begin_angle,end_angle); high = max(begin_angle,end_angle);
            extrema = (ceil(low/(pi/2)):floor(high/(pi/2)))*(pi/2);
            angles = [begin_angle,end_angle,extrema];
            points = [points;center+corner_radius*[cos(angles(:)),sin(angles(:))]]; %#ok<AGROW>
        end
        next_theta = theta+angle_change;
        state(1) = state(1)+signed_radius*(sin(next_theta)-s);
        state(2) = state(2)-signed_radius*(cos(next_theta)-c);
        state(3) = next_theta;
    end
    swept = [min(swept(1),min(points(:,1))),min(swept(2),min(points(:,2))), ...
        max(swept(3),max(points(:,1))),max(swept(4),max(points(:,2)))];
    detail.swept_bounds = swept;
    if swept(1)<bounds(1)-tol || swept(2)<bounds(2)-tol || ...
            swept(3)>bounds(3)+tol || swept(4)>bounds(4)+tol
        outside = true; detail.first_rejected_segment = segment; return;
    end
end
end
