classdef obstacle_cls_radar
% Description: 毫米波雷达类型
% Author :  Shane Liu
% Modify Note: 
% ************************************************************ %     
     enumeration
        pedestrian(1)       % 行人
        bicycle(2)          % 自行车
        car(3)              % 小汽车
        bus(4)              % 大巴
        leftGuard(5)        % 左护栏 
        rightGuard(6)       % 右护栏
    end

    methods(Static)
        function y = getDefaultValue()
            y = obstacle_cls.unknown;
        end
        
        function y = addClassNameToEnumNames()
            y = true;
        end
    end
end

