function linear_indices = st_vehicle_cells(map, pose, footprint, padding)
%ST_VEHICLE_CELLS 按分离轴定理栅格化旋转矩形与整个资源块的交集。
% 输入：地图、后轴位姿[x,y,theta]、膨胀矩形、扫掠补偿(m)。输出：二维线性索引。
% 四个分离轴为世界x/y及车身纵/横方向；接触也保守标为占用。
if nargin < 4, padding = 0; end
c = cos(pose(3)); s = sin(pose(3));
center = pose(1:2)+footprint.center_offset*[c,s];
hl = footprint.length/2+padding; hw = footprint.half_width+padding;
extent_x = hl*abs(c)+hw*abs(s)+map.dx/2;
extent_y = hl*abs(s)+hw*abs(c)+map.dy/2;
ix = find(abs(map.x_coords-center(1)) <= extent_x+1e-12);
iy = find(abs(map.y_coords-center(2)) <= extent_y+1e-12);
if isempty(ix) || isempty(iy), linear_indices = zeros(0,1); return; end
[X,Y] = ndgrid(map.x_coords(ix),map.y_coords(iy));
rx = X-center(1); ry = Y-center(2);
along = abs(rx*c+ry*s) <= hl+map.dx/2*abs(c)+map.dy/2*abs(s)+1e-12;
across = abs(-rx*s+ry*c) <= hw+map.dx/2*abs(s)+map.dy/2*abs(c)+1e-12;
[i,j] = find(along & across);
linear_indices = sub2ind([map.nx,map.ny],reshape(ix(i),[],1),reshape(iy(j),[],1));
end
