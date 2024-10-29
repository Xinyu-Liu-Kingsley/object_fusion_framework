classdef obj_exist_prop  < uint8
% Description: 目标存在概率
% Author :  Shane Liu
% Modify Note: 
% ************************************************************ % 
    enumeration
        percent_25 (0)       % 25
        percent_50 (1)       % 50
        percent_75 (2)       % 75
        percent_99 (3)       % 100
    end
    
    methods(Static)
        function y = getDefaultValue()
            y = obj_exist_prop.percent_25;
        end
        
        function y = addClassNameToEnumNames()
            y = true;
        end
    end
end

