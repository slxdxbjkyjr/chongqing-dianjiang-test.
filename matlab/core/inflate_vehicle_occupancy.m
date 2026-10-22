function footprint = inflate_vehicle_occupancy(vehicle_config, safety_distance)
%INFLATE_VEHICLE_OCCUPANCY 生成以后轴中心为参考的膨胀矩形。
% 输入：车辆尺寸/已有碰撞裕度、额外安全距离(m，默认0)。输出：矩形与包络。
% front/rear/half_width均已包含安全裕度；radius用于静态全航向保守膨胀。
if nargin < 2, safety_distance = 0; end
if ~isstruct(vehicle_config) || ~isfield(vehicle_config,'dimensions_m') || ...
        ~isnumeric(safety_distance) || ~isscalar(safety_distance) || ...
        ~isreal(safety_distance) || ~isfinite(safety_distance) || safety_distance < 0
    error('STOccupancy:InvalidVehicle', '车辆配置或额外安全距离无效。');
end
d = vehicle_config.dimensions_m;
if ~isstruct(d) || ~isscalar(d)
    error('STOccupancy:InvalidVehicle', '车辆尺寸必须为标量结构。');
end
fields = {'width','wheelbase','front_axle_to_front','rear_axle_to_rear'};
for k = 1:numel(fields)
    if ~isfield(d,fields{k})
        error('STOccupancy:InvalidVehicle', '缺少车辆尺寸字段%s。',fields{k});
    end
    value = d.(fields{k});
    if ~isnumeric(value) || ~isscalar(value) || ~isreal(value) || ~isfinite(value) || value < 0
        error('STOccupancy:InvalidVehicle', '车辆尺寸必须是非负有限标量。');
    end
end
if d.width <= 0 || d.wheelbase <= 0
    error('STOccupancy:InvalidVehicle', '车宽和轴距必须为正。');
end
margin = struct('front',0,'rear',0,'side',0);
if isfield(vehicle_config,'collision_margin_m'), margin = vehicle_config.collision_margin_m; end
if ~isstruct(margin) || ~isscalar(margin)
    error('STOccupancy:InvalidVehicle', '车辆裕度必须为标量结构。');
end
fields = {'front','rear','side'};
for k = 1:numel(fields)
    if ~isfield(margin,fields{k})
        error('STOccupancy:InvalidVehicle', '缺少车辆安全裕度字段%s。',fields{k});
    end
    value = margin.(fields{k});
    if ~isnumeric(value) || ~isscalar(value) || ~isreal(value) || ~isfinite(value) || value < 0
        error('STOccupancy:InvalidVehicle', '车辆安全裕度必须为非负有限实数标量。');
    end
end
front = d.wheelbase+d.front_axle_to_front+margin.front+safety_distance;
rear = d.rear_axle_to_rear+margin.rear+safety_distance;
half_width = d.width/2+margin.side+safety_distance;
footprint = struct('front',front,'rear',rear,'half_width',half_width, ...
    'length',front+rear,'width',2*half_width,'center_offset',(front-rear)/2, ...
    'radius',hypot(max(front,rear),half_width), ...
    'corners_local',[front,half_width;front,-half_width;-rear,-half_width;-rear,half_width]);
end
