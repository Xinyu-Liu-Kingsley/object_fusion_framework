classdef obstacle_cls < uint8
% Description: 障碍物类型
% Author :  Shane Liu
% Modify Note: 
% ************************************************************ % 
    enumeration
        unknown(0)              % 未知
        radarReflection(1)      % 雷达反射
        pedestrian(2)           % 行人
        bicycle(3)              % 骑行人
        bus(4)                  % 大巴车
        car(5)                  % 小汽车
        truck(6)                % 卡车
        generalObject(7)        % 通用障碍物
        animal(8)               % 自行车
        tinyCar(9)              % 小车
        uncertainVehicle(10)    % 异型车
        tricycle(11)            % 三轮车
        leftGuard(12)           % 左护栏
        rightGuard(13)          % 右护栏
        invalid(15)             % 无效
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

