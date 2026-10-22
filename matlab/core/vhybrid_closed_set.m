classdef vhybrid_closed_set < handle
    %VHYBRID_CLOSED_SET 保存五维连续状态索引和挡位，支持精确查重。

    properties (Access = private)
        keys_;
    end

    methods
        function obj = vhybrid_closed_set()
            % 构造空 Closed 集合；键包含 x/y/yaw/v/time 五个索引。
            obj.keys_ = containers.Map('KeyType', 'char', 'ValueType', 'logical');
        end

        function add(obj, node)
            %ADD 将节点加入 Closed 集合，并记录其节点编号。
            obj.keys_(obj.key(node)) = true;
        end

        function flag = contains(obj, node)
            %CONTAINS 判断五维离散状态是否已经访问过。
            flag = isKey(obj.keys_, obj.key(node));
        end

        function n = count(obj)
            %COUNT 返回已关闭状态数量。
            n = obj.keys_.Count;
        end
    end

    methods (Access = private)
        function key = key(~, node)
            %KEY 同位置/时间/速度的前进与倒车节点也不能错误合并。
            key = sprintf('%d_%d_%d_%d_%d_%d', node.x_index, node.y_index, ...
                node.yaw_index, node.velocity_index, node.time_index,node.gear);
        end
    end
end
