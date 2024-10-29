classdef obj_lane < uint8
% Description: 目标相对车道判断
% Author :  Shane Liu
% Modify Note: 
% ************************************************************ %     
    enumeration
        undefined (0)
        sameLane (1)
        leftLane (2)
        rightLane (3)
    end
    
    methods(Static)
        function y = getDefaultValue()
            y = obj_lane.undefined;
        end
        
        function y = addClassNameToEnumNames()
            y = true;
        end
    end
end

