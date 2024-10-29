classdef indicator_status < uint8

% Description: 转向指示灯状态 
% Author :  Shane Liu
% Modify Note: 
% ************************************************************ % 
    enumeration
        no_action(0)    % 无动作
        turn_l(1)       % 左转灯
        turn_r(2)       % 右转灯
        alarm(3)        % 警示、双闪
    end
    
    methods(Static)
        function y = getDefaultValue()
            y = indicator_status.no_action;
        end
        
        function y = addClassNameToEnumNames()
            y = true;
        end
    end
end

