classdef vhybrid_open_set < handle
    %VHYBRID_OPEN_SET 二叉最小堆，插入/取最小节点均为 O(log N)。
    % 相同 f 值按插入顺序稳定选择，保持原线性 min 的平局行为。

    properties (Access = private)
        nodes_;
        order_;
        sequence_ = 0;
        best_g_;
    end

    methods
        function obj = vhybrid_open_set()
            % 构造空 Open 集合。
            obj.nodes_ = repmat(vhybrid_node(), 0, 1);
            obj.order_ = zeros(0,1);
            obj.best_g_ = containers.Map('KeyType', 'char', 'ValueType', 'double');
        end

        function push(obj, node)
            %PUSH 将候选节点放入 Open 集合。
            obj.push_or_update(node);
        end

        function accepted = push_or_update(obj, node)
            %PUSH_OR_UPDATE 插入新状态或用更小 g 值替换旧候选。
            key = obj.key(node);
            accepted = ~(isKey(obj.best_g_, key) && obj.best_g_(key) <= node.g_cost);
            if ~accepted
                return;
            end
            obj.best_g_(key) = node.g_cost;
            obj.sequence_ = obj.sequence_+1;
            index = numel(obj.nodes_)+1;
            obj.nodes_(index,1) = node;
            obj.order_(index,1) = obj.sequence_;
            while index>1
                parent = floor(index/2);
                if ~obj.less(index,parent), break; end
                obj.swap(index,parent); index = parent;
            end
        end

        function [node, valid] = pop_min(obj)
            %POP_MIN 过期重复项惰性删除，不再线性扫描或递归取最小值。
            node = vhybrid_node(); valid = false;
            while ~isempty(obj.nodes_)
                candidate = obj.nodes_(1);
                count = numel(obj.nodes_);
                if count==1
                    obj.nodes_ = repmat(vhybrid_node(),0,1);
                    obj.order_ = zeros(0,1);
                else
                    obj.nodes_(1) = obj.nodes_(count);
                    obj.order_(1) = obj.order_(count);
                    obj.nodes_(count) = []; obj.order_(count) = [];
                    index = 1;
                    while 2*index<=numel(obj.nodes_)
                        child = 2*index;
                        if child+1<=numel(obj.nodes_) && obj.less(child+1,child), child = child+1; end
                        if ~obj.less(child,index), break; end
                        obj.swap(index,child); index = child;
                    end
                end
                key = obj.key(candidate);
                if isKey(obj.best_g_,key) && candidate.g_cost>obj.best_g_(key)+1e-12, continue; end
                node = candidate; valid = true; return;
            end
        end

        function flag = is_empty(obj)
            %IS_EMPTY 判断 Open 集合是否为空。
            flag = isempty(obj.nodes_);
        end

        function n = count(obj)
            %COUNT 返回 Open 集合当前节点数量。
            n = numel(obj.nodes_);
        end
    end

    methods (Access = private)
        function key = key(~, node)
            %KEY 挡位与五维离散索引共同决定搜索状态。
            key = sprintf('%d_%d_%d_%d_%d_%d', node.x_index, node.y_index, ...
                node.yaw_index, node.velocity_index, node.time_index,node.gear);
        end

        function flag = less(obj,a,b)
            fa = obj.nodes_(a).f_cost; fb = obj.nodes_(b).f_cost;
            flag = fa<fb || (fa==fb && obj.order_(a)<obj.order_(b));
        end

        function swap(obj,a,b)
            temporary = obj.nodes_(a); obj.nodes_(a) = obj.nodes_(b); obj.nodes_(b) = temporary;
            temporary_order = obj.order_(a); obj.order_(a) = obj.order_(b); obj.order_(b) = temporary_order;
        end
    end
end
