function tests = test_runtime_interface
%TEST_RUNTIME_INTERFACE 精简部署入口的最小回归测试。
% 这些测试只检查公共接口的输入、失败安全输出和文件副作用，
% 不把搜索器内部实现复制到测试中。
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
%SETUPONCE 让测试从任意当前目录都能找到部署接口。
test_dir = fileparts(mfilename('fullpath'));
root_dir = fileparts(test_dir);
addpath(fullfile(root_dir,'matlab','interface'));
clear plan_summon_trajectory;
testCase.TestData.root_dir = root_dir;
end

function testMissingStartStateReturnsStableFailure(testCase)
% 缺少起点时应返回明确错误码和固定长度的零速度轨迹。
request = struct('goal_state',[8,10,0,0,0]);
plan = plan_summon_trajectory(request);

verifyFalse(testCase,plan.valid);
verifyEqual(testCase,plan.error_code,100);
verifyEqual(testCase,numel(plan.time),601);
verifyEqual(testCase,plan.time(2)-plan.time(1),0.1,'AbsTol',1e-12);
verifyEqual(testCase,max(abs(plan.speed)),0,'AbsTol',1e-12);
verifyTrue(testCase,all(plan.gear == 0));
end

function testInvalidSpeedReturnsSafeStop(testCase)
% 负速度输入非法；失败轨迹仍停在请求起点且速度恒为0。
request = struct('start_state',[3,10,0,-1,0], ...
    'goal_state',[8,10,0,0,0]);
plan = plan_summon_trajectory(request);

verifyFalse(testCase,plan.valid);
verifyEqual(testCase,plan.error_code,100);
verifyEqual(testCase,plan.x(1),3,'AbsTol',1e-12);
verifyEqual(testCase,plan.y(1),10,'AbsTol',1e-12);
verifyEqual(testCase,plan.yaw(1),0,'AbsTol',1e-12);
verifyEqual(testCase,max(abs(plan.speed)),0,'AbsTol',1e-12);
verifyTrue(testCase,all(plan.gear == 0));
end

function testNonzeroStartTimeIsRejected(testCase)
% 固定输出时间轴从t=0开始，非零起始时间必须显式拒绝。
request = struct('start_state',[3,10,0,0,1], ...
    'goal_state',[8,10,0,0,0]);
plan = plan_summon_trajectory(request);

verifyFalse(testCase,plan.valid);
verifyEqual(testCase,plan.error_code,100);
verifyTrue(testCase,contains(plan.message,'时间必须为0'));
verifyEqual(testCase,numel(plan.time),601);
end

function testNormalRequestEntersSearch(testCase)
% 使用短距离、无静态障碍场景确认公共入口确实调用了搜索器。
runtime = load_summon_runtime_config();
runtime.planner_config.vhybrid.max_search_nodes = 1000;
runtime.planner_config.vhybrid.max_search_time_s = 1.0;
request = struct('start_state',[3,10,0,0,0], ...
    'goal_state',[8,10,0,0,0], ...
    'map_config',runtime.map_config, ...
    'output_dt_s',0.1,'max_output_time_s',1.0);
plan = plan_summon_trajectory(request,runtime);

verifyTrue(testCase,isfield(plan,'search_statistics'));
verifyGreaterThanOrEqual(testCase,plan.search_statistics.expanded_nodes,1);
verifyTrue(testCase,isfinite(plan.computation_time_s));
verifyEqual(testCase,numel(plan.time),11);
verifyEqual(testCase,plan.time(end),1.0,'AbsTol',1e-12);
% 该短时域不足以覆盖5 m路径，接口必须返回安全停车而非截断假成功。
verifyFalse(testCase,plan.valid);
verifyEqual(testCase,max(abs(plan.speed)),0,'AbsTol',1e-12);
end

function testNoProcessFilesAreCreated(testCase)
% 对外入口不应产生PNG/GIF/MAT/JSON过程文件。
root_dir = testCase.TestData.root_dir;
output_dir = fullfile(root_dir,'outputs');
before = countProcessFiles(output_dir);

request = struct('goal_state',[8,10,0,0,0]);
plan = plan_summon_trajectory(request);

after = countProcessFiles(output_dir);
verifyFalse(testCase,plan.valid);
verifyEqual(testCase,after,before);
end

function count = countProcessFiles(output_dir)
%COUNTPROCESSFILES 统计部署输出目录中的过程文件数量。
count = 0;
if exist(output_dir,'dir') ~= 7
    return;
end
extensions = {'*.png','*.gif','*.mat','*.json'};
for k = 1:numel(extensions)
    count = count + numel(dir(fullfile(output_dir,'**',extensions{k})));
end
end
