classdef obs_det_prop  < uint8
% Description: 输出结果来源
% Author :  Shane Liu
% Modify Note: 
% ************************************************************ % 
   enumeration
        undefined (0)          % 未知
        sole_radar (1)       % 毫米波雷达
        sole_camera (2)      % 相机
        fused (3)            % 融合
    end
    
    methods(Static)
        function y = getDefaultValue()
            y = obs_det_prop.undefined;
        end
        
        function y = addClassNameToEnumNames()
            y = true;
        end
    end
    
end

