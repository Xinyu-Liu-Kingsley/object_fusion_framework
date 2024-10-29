classdef obstacle_motionStatus < uint8 
% Description: 目标运动状态
% Author :  Shane Liu
% Modify Note: 
% ************************************************************ % 
    enumeration
        unknown(0)              % 未知
        moving(1)               % 运动
        stationary(2)           % 静止
        stopped(3)              % 停止
        movingSlowly(4)         % 缓慢运动
        invalid(7)              % 无效
    end

    methods(Static)
        function y = getDefaultValue()
            y = obstacle_motionStatus.unknown;
        end

        function y = addClassNameToEnumNames()
            y = true;
        end
    end

end



