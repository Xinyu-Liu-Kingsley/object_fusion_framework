classdef comp_res < uint8
% Description: 俯仰角动态补偿结果
% Author :  Shane Liu
% Modify Note: 
% **************************************************************** %
    enumeration
        unsuccessful(0)     % 条件不满足，无法进行补偿
        unneeded(1)       	% 当前俯仰角满足要求，无需进行补偿
        positive(2)     	% 正向补偿
        negative(3)        	% 反向补偿
        abnormal(4)         % 异常
    end
    
    methods(Static)
        function y = getDefaultValue()
            y = comp_res.unsuccessful;
        end
        
        function y = addClassNameToEnumNames()
            y = true;
        end
    end

end

