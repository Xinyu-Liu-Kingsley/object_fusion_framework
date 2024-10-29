classdef obstacle_const
% Description: 航迹相关常量定义
% Author :  Shane Liu
% Modify Note: 
% ************************************************************ % 
    properties (Constant)
        max_obs = uint8(16);        % 单帧识别障碍物最大数量
        track_depth = uint8(30);   	% 障碍物跟踪深度
        track_width = uint8(64);   	% 障碍物跟踪数量
        track_width_radar = uint8(32);
        % 16 -> 64
        max_obs_camera = uint8(64); % 前视摄像头最多目标个数，Raimo10 && EQ216
        max_obs_radar_front = uint8(64);
        max_obs_radar = uint8(192);
        max_obs_fuse = uint8(192);   % temp camera+radar
        max_lane_num = uint8(4);
        track_autos = uint8(24);    % 车辆目标跟踪数量
        len_1st = uint8(3);
        len_2nd = uint8(30);

        defNominalErr = 1e4;   % 名义距离默认值
        maxNominalErr = 1;
        defTTC = 10.2;
        defTHW = 10.2;
       

        % 关联参数
        cfg = struct(...
            'deltaT', 1/20,...
            'min_conf', 0.1,...                     % 最小置信度
            'x_coverage', [-16, 16],...              % 纵向有效范围
            'y_coverage', [-100, 180],...             % 横向有效范围
            'ref_widths', [1.6,1.80,  2.45, 2.45, 2.45, 2.45,2.45,2.45,2.45,2.45, 1, 0.75, 1],... %'ref_widths', [1.6,1.80,  2.25, 2.35, 2.45, 2.35,2.35,2.45,2.55,2.55,  0.75, 1],参考宽度与细分类模型对应
            'y', [0   20  50  90  150 200],...      % 纵向距离表
            's_x', [0.12 0.15 0.15 0.15 0.12 0.1]*0.5,...   % scale x查表值
            's_y', [0.8 0.8 0.8 0.7 0.6 0.6],...            % scale y查表值
            'a_y', [5 8 10 15 20 25],...                   % 单位：m;关联纵向阈值查表
            'weight',[0.9,0.9,0.1,0.1],...                 % x,y,vx,vy 权重
            'varX', 1, ...
            'varY', 2, ...
            'Rx', 0.1, ...                          % 初始创建轨迹时纵向Rx(m) Rx initial;
            'Ry', 0.2, ...                          % 横向(m)
            'Rvx',0.001,...
            'Rvy',0.001,...
            'r_ratio', 0.035, ...                   % 纵向测距噪声比例
            'thres_veh_moving',1.5,...                % 动静判断车辆阈值
            'thres_ped_moving',0.5,...                % 动静判断行人阈值
            'CIPVRangeMax',150,...                  % CIPV筛选范围
            'thres_stable', int32(3),...            % stable状态阈值。CNT大于等于此值进入stable状态
            'thres_unstable', int32(2),...          % unstable状态阈值。LST大于等于此值进入unstable状态
            'thres_lost', int32(2),...              % lost状态阈值。LST大于等于此值则目标丢失
            'thres_IDassign',uint8(20),...          % CNT大于此值，可以用ID关联
            'EOut_vehs_num',uint8(20),...           % 车辆输出目标最多10个数；依协议而定可变更；
            'EOut_peds_num',uint8(8),...            % 行人输出目标最多8个数；依协议而定可变更；
            'upper_clsCNT',uint8(50),...            % 估计与检测目标类型一致计算阈值；
            'lower_cls1CNT', uint8(10),...          % 估计目标类型切换阈值；
            'thres_obsw',0.3...                     % 目标宽度相差阈值；
            );


        RadarObject = struct( ...
            'time_ns',uint64(0), ...
            'ID',uint16(0),... % 雷达ID
            'x',0,...
            'y',0,...
            'obj_range',0,... %目标径向距离
            'moveType',0,...  % 目标运动属性 0：未知；1：靠近；2：远离；3：相对雷达静止；
            'obj_speed',0,... % 目标径向速度
            'obj_warningStaus',0,...%报警等级
            'obsolute_res',0, ...   % 目标运动状态 0：运动；1：绝对静止
            'obj_trackStatus',0,... % 跟踪状态
            'obj_azimuth',0,...     %目标角度
            'cls',obstacle_cls.unknown, ... %目标类型 1：人；2：自行车，电瓶车；3：小车；4：大车；5:左护栏；6:右护栏；
            'SNR',0,...       %RCS
            'ax',0,...        %水平加速度
            'ay',0,...        %垂直加速度
            'velo_x',0,...    % 目标水平速度，右为正
            'velo_y',0,...    % 目标垂直速度，远离为正
            'obj_lane',obj_lane.undefined,... % 数据预处理的时候根据车道进行判断
            'det_src',det_src.unknown,...   % 雷达数据来源
            'obj_exist_prop',obj_exist_prop.percent_25,... %目标存在概率
            'flag',uint8(0) ...      %数据是否有效
            );

        CameraObject = struct( ...
            'time_ns',uint64(0), ...
            'ID',uint8(0),... % 摄像头ID
            'x', 0,...        % 横向距离
            'y', 0,...        % 纵向距离
            'CIPVFlag',uint8(0),... % 
            'cls',obstacle_cls.unknown, ... %目标类型
            'velo_x',0,...    % 目标水平速度，右为正
            'velo_y',0,...    % 目标垂直速度，远离为正
            'obj_width',0,... %目标宽度
            'obj_y_arel',0,...% 目标加速度
            'obj_det_prop',obs_det_prop.undefined,... % 目标融合状态 0: undefined 1: sole-radar 2: sole-camera 3: fused
            'obj_exist_prop',obj_exist_prop.percent_25,... %目标存在概率
            'obj_dyn_prop',0,...
            'obj_lane',obj_lane.undefined,...  % 0:undefined  1:same lane 2:left lane 3:right lane
            'obj_direction',obj_direction.same_direction,...0: same direction 1: reverse direction 2: crosswise direction 3: reserve
            'img_lane',obj_lane.undefined, ...  % 0:undefined  1:same lane 2:left lane 3:right lane
            'det_src',det_src.unknown,...   % 摄像头数据来源
            'flag',uint8(0) ...      %数据是否有效
            );

        FusedObject = struct( ...
            'versionNum',2.04,...
            'ID',uint8(0),...
            'fusedCamID',uint8(0),...   % 关联的CameraID
            'fusedRadID',uint16(0),...   % 关联的radarID
            'CIPV',uint8(0),...         % CIPV flag
            'CIPP',uint8(0),...         % CIPP flag
            'x', 0,...                  % 横向距离
            'y', 0,...                  % 纵向距离
            'TTC',0,...                 % TTC
            'cls',obstacle_cls.unknown, ... %目标类型
            'motionStatus',obstacle_motionStatus.unknown, ... % 目标运动状态
            'motionCategory',obstacle_motionCategory.UNDEFINED, ... % 运动状态类型
            'velo_x',0,...    % 目标水平速度，右为正
            'velo_y',0,...    % 目标垂直速度，远离为正
            'ax',0,...        %水平加速度
            'ay',0,...        %垂直加速度
            'obj_width',0,... %目标宽度
            'moveType',0,...  % 目标运动属性 0：未知；1：靠近；2：远离；3：相对雷达静止；
            'obsolute_res',0,... %目标运动状态 0：运动；1：绝对静止
            'x_error',0,...
            'y_error',0,...
            'obj_det_prop',obs_det_prop.undefined,... % 目标融合状态 0: undefined 1: sole-radar 2: sole-camera 3: fused
            'obj_exist_prop',obj_exist_prop.percent_25,... % 目标存在概率（可以认为是置信度的概念）
            'obj_dyn_prop',0,...
            'obj_lane',obj_lane.undefined,...  % 0:undefined  1:same lane 2:left lane 3:right lane
            'obj_direction',obj_direction.same_direction,...0: same direction 1: reverse direction 2: crosswise direction 3: reserve
            'img_lane',obj_lane.undefined, ...  % 0:undefined  1:same lane 2:left lane 3:right lane
            'state',track_status.untracking,...
            'det_src',det_src.unknown,...   % 融合数据来源
            'flag',uint8(0) ...      %数据是否有效
            );



        %每一帧数据包
        CameraFrame = struct( ...
            'time_ns',uint64(0), ...            % 摄像头时间戳...
            'CameraObjectList',repmat(obstacle_const.CameraObject, 1, obstacle_const.max_obs_camera),...
            'LaneList',repmat(lane_const.LaneXY,1,lane_const.max_lane_num)...
            );
        RadarFrame = struct( ...
            'time_ns',uint64(0), ...            % 雷达时间戳...
            'RadarObjectList',repmat(obstacle_const.RadarObject, 1, obstacle_const.max_obs_radar)...
            );
        % TODO 改
        FrameData = struct( ...
            'CameraFrame',obstacle_const.CameraFrame,...
            'RadarFrame',obstacle_const.RadarFrame...
            );



        % 障碍物轨迹库
        MatObstacles = struct(...
            'CNT', int32(0), ...       	% 计数器
            'cursor', uint8(1), ...    	% 轨迹库游标（初始值为1！！！）
            'matrix', repmat(obstacle_const.FusedObject, obstacle_const.track_depth, obstacle_const.track_width), ... % 障碍物轨迹库
            'lstflg', zeros(obstacle_const.track_depth, obstacle_const.track_width, 'uint8'), ...
            'status', repmat(struct( ...
            'CNT', int32(0), ...     % 生命计数器
            'LST', int32(0), ...     % 丢帧计数器
            'CNTCam',int32(0),...    % Camera计数器
            'CNTRad',int32(0),...    % Radar计数器
            'LSTCam',int32(0),...    % Camera丢帧计数器
            'LSTRad',int32(0),...    % Radar丢帧计数器
            'CNTFused',int32(0),...  % 融合状态计数器
            'fusedCls',obstacle_cls.unknown,...  % 融合目标类型
            'motionStatus',obstacle_motionStatus.unknown,...       % 融合目标运动状态
            'motionCategory',obstacle_motionCategory.UNDEFINED,... %
            'start_Range',0,...      % 航迹起始距离
            'flag', uint8(0), ...    % 当前更新标识。0 - 表示cursor指向的位置并未更新数据；1 - 表示cursor指向的位置已更新数据。每一次都会在updateTracksObstacle函数中被置位
            'used', uint8(0), ...    % 占用标识。0 - 表示该条轨迹未被使用；1 - 表示该条轨迹正在被使用
            'state', track_status.untracking, ...            % 跟踪状态
            'CamID_history', zeros(10,1,'uint8') ,...        % 关联CamID历史值
            'RadID_history',zeros(10,1,'uint16'),...          % 关联RadID历史值
            'y_error_history',zeros(6,1),....               % 纵向关联误差
            'x_error_history',zeros(6,1),...                % 横向关联误差
            'matchErrorFlag',uint8(0),...                    % 判断关联误差是否增大
            'y_history',zeros(10,1),...                      % 纵向距离历史值
            'x_history',zeros(10,1),...                      % 横向距离历史值
            'vy_history',zeros(10,1),...                     % 纵向速度历史值
            'vx_history',zeros(10,1),...                     % 横向速度历史值
            'rel_status',uint8([0 0]) ...                    % cnt &status 纵向状态：0:unknown; 1,目标车静止；2，目标车速小于自车速deltaD<0；3，目标车速大于等于自车速deltaD>0；
            ), 1, obstacle_const.track_width), ...           % 轨迹状态
            'x_est', zeros(6,1,obstacle_const.track_width), ...
            'p_est', zeros(6,6,obstacle_const.track_width), ...
            'idxCIPV', uint8(0), ...
            'estimated',    repmat(obstacle_const.FusedObject, 1, obstacle_const.track_width), ...   % 估计下一帧目标状态。每一次都会在predictTracksObstacle函数中被更新
            'output',       repmat(obstacle_const.FusedObject, 1, obstacle_const.track_width) ...	% 当前有效目标输出。每一次都会在updateTracksObstacle函数中被更新
            );
    end

end

