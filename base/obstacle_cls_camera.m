classdef obstacle_cls_camera < uint8
% Description: 相机类型
% Author :  Shane Liu
% Modify Note: 
% ************************************************************ % 
    enumeration
        car(0)              % 一般轿车，小轿车、SUV、面包车等中型轿车，以及大型轿车，多数位于此区间
        truck(1)            % 重卡， 单侧三轮以上卡车
        pedestrian(2)       % 行人
        bus(3)              % 大巴车
        bicycle(4)          % 自行车
        unknown(5)
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

