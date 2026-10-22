function runtime = load_summon_runtime_config(config_dir)
%LOAD_SUMMON_RUNTIME_CONFIG 读取召集规划运行时配置。
% 输入：
%   config_dir - 可选配置目录；省略时使用本文件上两级目录下的config。
% 输出：
%   runtime.vehicle_config - 车辆尺寸、轴距、转角和碰撞裕度。
%   runtime.map_config     - 召集区域边界和静态障碍物配置。
%   runtime.planner_config  - V-Hybrid A*、动力学和时空地图配置。
%   runtime.root_dir        - 10.18test根目录，供接口定位文件使用。
%
% 说明：JSON只在MATLAB启动或接口初始化阶段读取。MATLAB Function
% Block运行时应把解析后的三个配置结构体作为输入传入，避免在块内调用
% fileread/jsondecode。

if nargin < 1 || isempty(config_dir)
    interface_dir = fileparts(mfilename('fullpath'));
    matlab_dir = fileparts(interface_dir);
    root_dir = fileparts(matlab_dir);
    config_dir = fullfile(root_dir,'config');
else
    config_dir = char(config_dir);
    root_dir = fileparts(config_dir);
end

required_files = {'vehicle_config.json','map_config.json','planner_config.json'};
for k = 1:numel(required_files)
    file_path = fullfile(config_dir,required_files{k});
    if exist(file_path,'file') ~= 2
        error('Summon:MissingConfig','找不到配置文件：%s。',file_path);
    end
end

runtime = struct();
runtime.root_dir = root_dir;
runtime.config_dir = config_dir;
runtime.vehicle_config = jsondecode(fileread(fullfile(config_dir,'vehicle_config.json')));
runtime.map_config = jsondecode(fileread(fullfile(config_dir,'map_config.json')));
runtime.planner_config = jsondecode(fileread(fullfile(config_dir,'planner_config.json')));
end
