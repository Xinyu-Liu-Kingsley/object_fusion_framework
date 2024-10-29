classdef wiper_status < uint8
% Description: 雨刷器状态
% Author :  Shane Liu
% Modify Note: 
% ************************************************************ % 
    enumeration
        no_action(0)    % 无动作
        interval(1)     % 间歇
        slow(2)         % 慢速
        fast(3)         % 快速
    end
    
    methods(Static)
        function y = getDefaultValue()
            y = wiper_status.no_action;
        end
        
        function y = addClassNameToEnumNames()
            y = true;
        end
    end
end

