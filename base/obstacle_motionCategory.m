classdef obstacle_motionCategory < uint8
% Description: 目标运动状态分类
% Author :  Shane Liu
% Modify Note: 
% ************************************************************ %     
    enumeration
        UNDEFINED (0)
        PASSING   (1)         %超车（旁车道）
        PASSING_IN (2)        %向内侧超车（旁车道）
        PASSING_OUT (3)       %向外侧超车（旁车道）
        CLOSE_CUT_IN (4)      %靠近其他车辆并向内切
        MOVING_IN (5)         %靠近
        MOVING_OUT (6)        %远离
        CROSSING (7)          %横穿
        LTAP (8)              %左转向操作
        RTAP (9)              %右转向操作
        MOVING (10)           %车辆移动
        PRECEEDING (11)       %跟随前车中（同车道）
        ONCOMING (12)         %对向靠近中（对向车道）     
        INVALID (15)          %无效
    end

    methods(Static)
        function y = getDefaultValue()
            y = obstacle_motionCategory.UNDEFINED;
        end

        function y = addClassNameToEnumNames()
            y = true;
        end
    end
end

