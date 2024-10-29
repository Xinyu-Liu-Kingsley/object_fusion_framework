classdef obj_direction < uint8
% Description: 相对方向判断
% Author :  Shane Liu
% Modify Note: 
% ************************************************************ % 
    enumeration
        same_direction (0)
        reverse_direction (1)
        crosswise_direction (2)
        reserve (3)
    end

  methods(Static)
        function y = getDefaultValue()
            y = obj_direction.same_direction;
        end
        
        function y = addClassNameToEnumNames()
            y = true;
        end
    end
end

