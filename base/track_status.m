classdef track_status < uint8
% Description: 目标跟踪状态
% Author :  Shane Liu
% Modify Note: 
% ************************************************************ % 
    enumeration
        untracking  (0)     % 未跟踪
        initial     (1) 	% 初始过程
        growing     (2)  	% 建立过程
        stable      (3)  	% 稳定跟踪
        unstable    (4)     % 不稳定
    end
    
    methods(Static)
        function y = getDefaultValue()
            y = track_status.untracking;
        end
        
        function y = addClassNameToEnumNames()
            y = true;
        end
    end
    
end

