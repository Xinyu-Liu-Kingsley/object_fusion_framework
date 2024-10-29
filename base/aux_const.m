classdef aux_const
% Description: 定义全局常量
% Author :  Shane Liu
% Modify Note: 
% **************************************************************** %
    properties (Constant)
        max_frames = uint32(6000);  % 单个数据文件保存最多帧数
        factor4storing = 1e4;   
        factor4loading = 1e-4;
    end
    
end

