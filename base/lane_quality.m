classdef lane_quality < uint8
% Description: 车道线类型
% Author :  Shane Liu
% Modify Note: 
% ************************************************************ % 
    enumeration
        low_quality(0)          % 低置信度
        high_quality(1)         % 高置信度
        other(3)
    end
    
    methods(Static)
        function y = getDefaultValue()
            y = lane_quality.other;
        end
        
        function y = addClassNameToEnumNames()
            y = true;
        end
    end
    
end

