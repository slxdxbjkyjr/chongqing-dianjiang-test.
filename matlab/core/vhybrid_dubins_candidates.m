function curves = vhybrid_dubins_candidates(start, goal, radius)
%VHYBRID_DUBINS_CANDIDATES 在单位半径坐标下求 LSL/RSR/LSR/RSL/RLR/LRL。
% L/R 的长度是非负转角，S 是直线长度；乘 radius 后全部为弧长 m。
% 六类公式由终点连接与启发式共用，输出按总弧长升序排列。
% 输入start/goal为[x,y,theta]，radius为车辆最小转弯半径；距离单位m。
if ~isnumeric(radius) || ~isscalar(radius) || ~isfinite(radius) || radius <= 0
    error('VHybrid:InvalidTurningRadius','最小转弯半径必须为正有限标量。');
end
d = hypot(goal(1)-start(1),goal(2)-start(2))/radius;
bearing = atan2(goal(2)-start(2),goal(1)-start(1));
alpha = mod(start(3)-bearing,2*pi); beta = mod(goal(3)-bearing,2*pi);
sa = sin(alpha); sb = sin(beta); ca = cos(alpha); cb = cos(beta);
cab = cos(alpha-beta);
lengths = nan(6,3);
types = [1,0,1; -1,0,-1; 1,0,-1; -1,0,1; -1,1,-1; 1,-1,1];
p2 = 2+d*d-2*cab+2*d*(sa-sb);
if p2 >= -1e-12
    tmp = atan2(cb-ca,d+sa-sb);
    lengths(1,:) = [mod(-alpha+tmp,2*pi),sqrt(max(0,p2)),mod(beta-tmp,2*pi)];
end
p2 = 2+d*d-2*cab+2*d*(sb-sa);
if p2 >= -1e-12
    tmp = atan2(ca-cb,d-sa+sb);
    lengths(2,:) = [mod(alpha-tmp,2*pi),sqrt(max(0,p2)),mod(-beta+tmp,2*pi)];
end
p2 = -2+d*d+2*cab+2*d*(sa+sb);
if p2 >= -1e-12
    p = sqrt(max(0,p2));
    tmp = atan2(-ca-cb,d+sa+sb)-atan2(-2,p);
    lengths(3,:) = [mod(-alpha+tmp,2*pi),p,mod(-beta+tmp,2*pi)];
end
p2 = d*d-2+2*cab-2*d*(sa+sb);
if p2 >= -1e-12
    p = sqrt(max(0,p2));
    tmp = atan2(ca+cb,d-sa-sb)-atan2(2,p);
    lengths(4,:) = [mod(alpha-tmp,2*pi),p,mod(beta-tmp,2*pi)];
end
tmp = (6-d*d+2*cab+2*d*(sa-sb))/8;
if abs(tmp) <= 1+1e-12
    p = mod(2*pi-acos(max(-1,min(1,tmp))),2*pi);
    t = mod(alpha-atan2(ca-cb,d-sa+sb)+p/2,2*pi);
    lengths(5,:) = [t,p,mod(alpha-beta-t+p,2*pi)];
end
tmp = (6-d*d+2*cab+2*d*(-sa+sb))/8;
if abs(tmp) <= 1+1e-12
    p = mod(2*pi-acos(max(-1,min(1,tmp))),2*pi);
    t = mod(-alpha-atan2(ca-cb,d+sa-sb)+p/2,2*pi);
    lengths(6,:) = [t,p,mod(beta-alpha-t+p,2*pi)];
end
valid_ids = find(all(isfinite(lengths),2));
[~,order] = sort(sum(lengths(valid_ids,:),2));
curves = repmat(struct('lengths',zeros(1,3),'types',zeros(1,3)),numel(order),1);
for k = 1:numel(order)
    id = valid_ids(order(k));
    curves(k).lengths = radius*lengths(id,:);
    curves(k).types = types(id,:);
end
end
