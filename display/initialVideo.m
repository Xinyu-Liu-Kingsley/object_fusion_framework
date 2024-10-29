% 读取数据
video_path = [dat_path, 'video'];
files_blf = dir(fullfile(video_path, '*.avi'));
[~, blf_name, ~] = fileparts(files_blf(x).name);
frameData_name = strcat(blf_name,"_frmData");
carSignals_name = strcat(blf_name,"_carSignals");
load(fullfile(raw_path, frameData_name));
load(fullfile(raw_path, carSignals_name));
if displayMDCU == 1
    MDCU_name = strcat(blf_name,"_MDCUfrmData");
    load(fullfile(raw_path,MDCU_name));
end
frameData = frmData;
% 修改离线测试数据运行的帧数，生成代码时建议改成1 %
% frmID_end=int32(length(frameData));
frmID_end=int32(10);

if x~=videoNo(1)
    vidfile = files(x).name;
    % 获取文件名
    [~, name, ~] = fileparts(vidfile);
    % 读取视频
    vid = VideoReader(fullfile(vid_path, vidfile));
    frmID0 = int32(1);
else
    frmID0 = frmID_start;
end
%===============保留连续时间内的CIPV轨迹状态；=================
cipvTrack = zeros(aux_const.max_frames,14);      % [id x y vx vy cls states] add [direction cls1][TTC rel_status][lstmX lstmVx][EKFX EKFVx]
lTrack = zeros(aux_const.max_frames,6);          % [id x y vx vy states]
rTrack = zeros(aux_const.max_frames,6);          % [id x y vx vy states]
obsCIPV = zeros(aux_const.max_frames,13);        % [x,y,x1,x2,u,v,w,h,r,cls]; add 修改后的 [x3 obsW1 obsW2]
obsL = zeros(aux_const.max_frames,13);           % [x,y,x1,x2,u,v,w,h,r,cls]; add 修改后的 [x3 obsW1 obsW2]
obsR = zeros(aux_const.max_frames,13);           % [x,y,x1,x2,u,v,w,h,r,cls]; add 修改后的 [x3 obsW1 obsW2]
CLeftMatrix = zeros(aux_const.max_frames,11);    % [C0-C3, filter C0-C3,cls,conf, wheel_y]
CRightMatrix =  zeros(aux_const.max_frames,11);  % [C0-C3, filter C0-C3,cls,conf, wheel_y]
laneParams = zeros(aux_const.max_frames,4);      % [laneWidth radius yawrate lateral_speed]
cameracipv = zeros(aux_const.max_frames,6);      % [id x y vx vy cls states]
radarcipv = zeros(aux_const.max_frames,5);
cippTrack = zeros(aux_const.max_frames,10);      % [id x y vx xy state]
obsCIPP = zeros(aux_const.max_frames,6);         % [x,u v,w,h,cls]

% 车道线重合度量值
lanes_uv_var = zeros(aux_const.max_frames,4);    %  left [means std] right [means std]
%=====================保存雷达轨迹========================
frmN = 6000;
tracksN = 100;
s = struct('id',uint8(0),'measures',zeros(frmN,6,'single'),'cnt',uint16(0),'interpFlag',uint8(0)); %[time x y vx vy RCS]
radarTracksMat = repmat(s,tracksN,1);
cRadar = struct('id',uint8(0),'data',zeros(frmN,7,'single'));  % [time,id x,y,vx,vy,RCS]
cippRadar = struct('id',uint8(0),'data',zeros(frmN,7,'single'));
lRadar = struct('id',uint8(0),'data',zeros(frmN,7,'single'));
rRadar = struct('id',uint8(0),'data',zeros(frmN,7,'single'));
radarFlag = false;
% 创建画布
if displayFlag
    h = figure('color', 'w', 'position', [10, 10, 1800, 1070], 'name', 'camera', 'visible', 'on');
    % 显示毫米波数据
    column_name_radar =  {...
        'ID_radar','x','y','cls','velo_x','velo_y'};
    % 显示摄像头数据
    column_name_camera = {...
        'ID_camera','x','y','cls', 'velo_x', 'velo_y'};
    % 显示融合数据
    %     column_name_fused = {...
    %         'ID_fused','CamID','RadID','x','y','cls','velo_x','velo_y','ax','ay','width','moveType',...
    %         'det_prop','exist_prop','dyn_prop','obj_lane','direction','img_lane','state','flag',
    %         };
    column_name_fused = {...
        'ID_fused','CamID','RadID','CIPV','CIPP','x','y','cls','motSta', ...
        'motCate','velo_x','velo_y','width','obj_lane','img_lane','MoveType','ObsRes','TTC','detSrc'};
    rowNames1 = {};
    data1 = {};
    uitableFused = uitable('Data', data1, 'ColumnName', column_name_fused, 'RowName', rowNames1, 'Units', 'normalized', 'Position', [0.01 0.05 0.75 0.28]);
    data2 = {};
    uitableCamera = uitable('Data', data2, 'ColumnName', column_name_camera, 'RowName', rowNames1, 'Units', 'normalized', 'Position', [0.75 0.7 0.25 0.28]);
    data3 = {};
    uitableRadar = uitable('Data', data3, 'ColumnName', column_name_radar, 'RowName', rowNames1, 'Units', 'normalized', 'Position', [0.75 0.01 0.25 0.65]);

    ax1 = axes;
    ax1.Position = [0.01,0.4,0.4,0.58];
    %     title(['frmIdx = ',num2str(frmIDs(i))]);
    % 隐藏坐标轴的相对位置
    set(ax1, 'XLabel', [], 'YLabel', []);
    ax2 = axes;
    ax2.Position = [0.44,0.4,0.26,0.575];
    set(ax2, 'XLabel', [], 'YLabel', []);
    %     title('CameraResult')
    %     ax1 = axes;
    %     ax1.Position = [0.01,0.4,0.4,0.58];
    %     x2 = 1:10;
    %     y2 = x2.^3;
    %     plot(x2, y2);
    %     ax2 = axes;
    %     ax2.Position = [0.44,0.4,0.3,0.58];
    %     x1 = 1:10;
    %     y1 = x1.^2;
    %     plot(x1, y1);
end
% 确定帧号
frmID1 = frmID_end;
%frmID1 = int32(200);
frmIDs = frmID0:frmID1;
% 瞬态校正数组
pitches = zeros(1);
% 常量验证
% success = mbdAssertConst(lane_const.max_lns, lane_const.max_pts, lane_const.max_NNs, ...
%     lane_const.track_depth, lane_const.track_width, obstacle_const.max_obs, ...
%     obstacle_const.track_depth, obstacle_const.track_width, cam_const.NFS, aux_const.max_frames);

% 读取数据

% [lanes_nn1, lanes_nn2, vehs_uv1,vehs_uv2,peds_uv1,peds_uv2,cropSizeObs,cropSizePeds,cropSizeLane,laneCrossSeg,...
% num_frm,carSignals, soft_version,error,video_offset] = auxLoadData3([dat_path, name], frmID0, frmID1);
%

% if num_frm ~= length(frmIDs)
%     frmIDs = frmID0 + int32(0:num_frm-1);
% end
% 读取radar数据
% if exist(fullfile(dat_path, [name, '_radar.mat']), 'file')
%     load(fullfile(dat_path, [name, '_radar.mat']));
%     radFrm = numel(radArray);
%     if radFrm>0 ;radarFlag = true; end
% else
%     radArray=[];
%     radFrm = 0;
% end
% 判断视频与雷达相差帧数；
% if radFrm~=0
%     offsetFrm = vid.NumFrames-radFrm;
% else
%     offsetFrm = 0;
% end
% 存储雷达航迹
% if isempty(radArray)
%     radTable = [];
% else
%     radTable = struct2table(radArray);
%     T = radTable.tm;
%     for i = 1:radFrm
%         time = radArray(i).tm;
%         radTar = radArray(i).frame;
%         [radarTracksMat] = saveRadarTracks(radarTracksMat,i,radTar,time);
%     end
% end
% 读取车速信息
% carSignalArray = zeros(aux_const.max_frames,3);
% if exist (fullfile(dat_path, 'mat',[name, '_car_signals.mat']), 'file')
%     load(fullfile(dat_path, 'mat',[name, '_car_signals.mat']));
%     [carSignalFrm,~] = size(car_signals);
%     for i = 1:carSignalFrm
%         carSignalArray(i,1) = car_signals{i,1};
%         carSignalArray(i,2) = car_signals{i,2};
%         carSignalArray(i,3) = car_signals{i,3};
%     end
%     carSignalFlag = true;
% else
%     carSignalFlag = false;
% end

% 视频参数
vid.CurrentTime = (double(frmIDs(1))-1)/vid.FrameRate;
if saveVideo
    [~, name, ~] = fileparts(vidfile);
    cd([dat_path, 'fusion_results']);
    v_res = VideoWriter([name, '_res'], 'MPEG-4');
    v_res.FrameRate = vid.FrameRate;
    open(v_res);
end