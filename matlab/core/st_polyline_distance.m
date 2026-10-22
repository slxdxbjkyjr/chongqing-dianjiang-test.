function distance = st_polyline_distance(X, Y, polygon)
%ST_POLYLINE_DISTANCE 计算各点到闭合多边形所有线段的最短距离。
% 输入：同尺寸X/Y坐标与N×2多边形。输出：同尺寸距离(m)。
% 逻辑：投影到线段并截断到[0,1]，避免只检查顶点而漏掉长边中部。
distance = inf(size(X));
if any(polygon(1,:) ~= polygon(end,:)), polygon(end+1,:) = polygon(1,:); end
for k = 1:size(polygon,1)-1
    a = polygon(k,:); b = polygon(k+1,:); direction = b-a;
    length_squared = sum(direction.^2);
    if length_squared == 0
        candidate = hypot(X-a(1),Y-a(2));
    else
        fraction = ((X-a(1))*direction(1)+(Y-a(2))*direction(2))/length_squared;
        fraction = max(0,min(1,fraction));
        candidate = hypot(X-a(1)-fraction*direction(1),Y-a(2)-fraction*direction(2));
    end
    distance = min(distance,candidate);
end
end
