classdef ego_const
% Description: 自车相关参数
% Author :  Shane Liu
% Modify Note: 
% ************************************************************ % 
    properties (Constant)
        min_axles = uint8(2);           % 最少车轴数
        max_axles = uint8(4);           % 最多车轴数
        rng_width = [1.5, 2.8];         % 宽度范围
        rng_length = [4, 20];           % 长度范围
        rng_height = [1.4, 4];          % 高度范围
        rng_wheelbase = [1.2, 2.8];     % 轮距范围
        rng_foverhang = [0.5, 2];       % 前悬范围
        rng_axlebase = [1.2, 8];
        rng_mass = [3000, 100000];
        rng_axleweight = [100, 30000];
        len_vx_history = uint8(10);
        len_yaw_history = uint8(7);
        
        rng_speed = [-50, 150];         % 车速范围
        rng_vx = [-20, 50];             % vx范围
        rng_vy = [-10, 10];             % vy范围
        rng_ax = [-10, 10];             % ax范围
        rng_ay = [-10, 10];             % ay范围
        rng_SW_angle = [-720, 720];     % 方向盘转角范围
        rng_SW_alpha = [-720, 720];     % 方向盘转速范围
        rng_yawrate = [-100, 100];      % 横摆角范围
        rng_BPP = [0, 100];             % 制动踏板范围
        rng_APP = [0, 100];             % 油门踏板范围
        
        params = struct(...
            'axle_nums', uint8(2),...   % 轴数
            'width', 2.5,...            % 宽，单位：m
            'length', 8.0,...           % 长，单位：m
            'height', 3.0,...           % 高，单位：m
            'front_overhang', 1.0,...   % 前悬，单位：m
            'front_wheelbase', 2.3,...  % 前轴轮距，单位：m
            'axle12_base', 0,...        % 1-2轴距，单位：m
            'axle23_base', 0,...        % 2-3轴距，单位：m
            'axle34_base', 0,...        % 3-4轴距，单位：m
            'total_mass', 20000,...     % 高，单位：m
            'curb_mass', 10000,...      % 整备质量，单位：kg
            'axle1_weight', 0,...       % 1轴轴荷，单位：kg
            'axle2_weight', 0,...       % 2轴轴荷，单位：kg
            'axle3_weight', 0,...       % 3轴轴荷，单位：kg
            'axle4_weight', 0,...       % 4轴轴荷，单位：kg
            'rear_overhang', 0 ...      % 后悬，单位：m
            );
        status = struct(...
            'speed', 0,...              % 车速，单位：km/h
            'vx', 0,...                 % 纵向速度，单位：m/s
            'vx_history',zeros(ego_const.len_vx_history,1),...% 历史车速 10 frames,unit:m/s
            'vy', 0,...                 % 横向速度，单位：m/s
            'ax', 0,...                 % 纵向加速度，单位：m/s/s
            'ay', 0,...                 % 横向加速度，单位：m/s/s
            'SW_angle', 0,...           % 方向盘转角，单位：deg
            'SW_alpha', 0,...           % 方向盘转速，单位：deg/s
            'yawrate', 0,...            % 横摆角速率，单位：deg/s
            'yawrateHis',zeros(ego_const.len_yaw_history,1),...  % 横摆角速率历史信息，单位：deg/s
            'cursor', uint8(1), ...    	% 游标（初始值为1）
            'BPS', uint8(0),...         % 制动踏板开关，0/1
            'BPP', 0,...                % 制动踏板位置，0-100，单位：%
            'APP', 0,...                % 油门踏板位置，0-100，单位：%
            'wiper', wiper_status.no_action,...         % 雨刮器状态
            'indicator', indicator_status.no_action,... 	% 转向灯状态
            'curvature', 0 ... 	        % 曲率    在车道线使用时记得除以2      
            );
    end
    
end

