% Description: 代码生成工程获取输入类型定义demo
% log:
% 注意事项：在对外输入输出存在变更时应同步更新本脚本并在代码生成工程中执行！！
%*************************************************************************%
close all
clear
clc

% 获取源数据路径
% [raw_path,dat_path,vid_path] = getPath();
get_local_path_const();
vidfiles = dir(fullfile(vid_path, '*.mp4'));

% 定义全局变量
create_global_vars;

global g_ego_params g_cfg_lane g_cfg_auto;

% 配置显示信息
mbdCfgDispInfo(info_cls.anything, info_spec.general);


%%
vidfile = vidfiles(1).name;

% 文件名
[~, name, ~] = fileparts(vidfile);

vid = VideoReader(fullfile(vid_path, vidfile));

frmID0 = int32(450);
% frmID1 = int32(2000);
frmID1 = int32(451);
frmIDs = frmID0:frmID1;
pitches = zeros(1);

% 车辆
mbdInitializeEgo(g_ego_params.axle_nums, g_ego_params.width, ...
    g_ego_params.length, g_ego_params.height, g_ego_params.front_overhang, ...
    g_ego_params.front_wheelbase,g_ego_params.axle12_base, ...
    g_ego_params.axle23_base, g_ego_params.axle34_base, ...
    g_ego_params.total_mass, g_ego_params.curb_mass, ...
    g_ego_params.axle1_weight, g_ego_params.axle2_weight, ...
    g_ego_params.axle3_weight, g_ego_params.axle4_weight);

% 车道线
mbdInitializeLane(vid.FrameRate, g_cfg_lane.min_conf, g_cfg_lane.min_pts,...
    g_cfg_lane.x_coverage, g_cfg_lane.y_coverage,...
    g_cfg_lane.w_coverage, g_cfg_lane.min_x_len, g_cfg_lane.max_x_near, ...
    g_cfg_lane.max_heading, g_cfg_lane.min_radius, ...
    g_cfg_lane.def_wid_half, g_cfg_lane.max_lst, g_cfg_lane.min_cnt, ...
    g_cfg_lane.max_delta_u, g_cfg_lane.max_delta_v,...
    g_cfg_lane.epsilon, g_cfg_lane.min_clu);

% 障碍物
mbdInitizlizeObstacle(vid.FrameRate, g_cfg_auto.min_conf, ...
    g_cfg_auto.x_coverage, g_cfg_auto.y_coverage, ...
    g_cfg_auto.ref_widths, g_cfg_auto.thres_stable, ...
    g_cfg_auto.thres_unstable, g_cfg_auto.thres_lost);

% 常量验证
success = mbdAssertConst(lane_const.max_lns, lane_const.max_pts, lane_const.max_NNs, ...
    lane_const.track_depth, lane_const.track_width, obstacle_const.max_obs, ...
    obstacle_const.track_depth, obstacle_const.track_width, cam_const.NFS, aux_const.max_frames);

% 初始化lstm模型参数
% global lstm_params
% lstm_params = load('E:\ProjectCode\Postprocessing\自研\hh_LSTM\CMBD\scripts\networkModel\lstm.mat');
% [lstm_weights,lstm_mem_dict] = mbdintitizlizeLSTMparams(lstm_params); 


% 创建摄像头
X = -1.6;
Y = 0;
Z = 1.5;
pitch = -1.5;
yaw = 0.6;
roll = 0;
veh_wid = 1.8;
if mbdCreateCamera((1824.94313), (1821.13332), (956.27022), (567.59414), uint32([1080, 1920]), Z, X, Y, pitch, yaw, roll) ~= uint8(0)
    error('create camera error!')
end

[lanes_nn1, lanes_nn2, vehs_uv1,vehs_uv2,peds_uv1,peds_uv2,cropSizeObs,cropSizePeds,cropSizeLane,laneCrossSeg,...
num_frm,carSignals, soft_version,error,video_offset] = auxLoadData3([dat_path, name], frmID0, frmID1);

if num_frm ~= length(frmIDs)
    frmIDs = frmID0 + int32(0:num_frm-1);
end

% 逐帧处理车道线、障碍物结果
vid.CurrentTime = (double(frmIDs(1))-1)/vid.FrameRate;
for i=1:length(frmIDs)
    if ~hasFrame(vid), break; end
    frm = readFrame(vid);

    mbdUpdateEgo(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, ...
        wiper_status.no_action, indicator_status.no_action);

    % 将神经网络的车道线检测结果进行聚类
    lanes_uv = mbdLaneNNClustering(lanes_nn1(frmIDs(i)), lanes_nn2(frmIDs(i)), laneCrossSeg(frmIDs(i), :));
    %     lanes_uv1 = mbdLaneNNClustering(lanes_nn1(frmIDs(i)));
    %     lanes_uv2 = mbdLaneNNClustering(lanes_nn2(frmIDs(i)));
    %     lanes_uv =  mbdLaneUVCombine(lanes_uv1,lanes_uv2);

    % 处理车道线检测点
    [lanes_me, lanes_xy,lanes_uv] = mbdProcessLanes2(lanes_uv);

    % 车道线检测结果输入车道线跟踪库
    lanes_eo = mbdAppendLanes(lanes_me);

    % 二阶段目标框选择
    vehicles_uv = repmat(obstacle_const.BBox, 1,100);
    for kk = 1:obstacle_const.max_obs
        vehicles_uv(kk).u = vehs_uv1(frmIDs(i), kk).u;
        vehicles_uv(kk).v = vehs_uv1(frmIDs(i), kk).v;
        vehicles_uv(kk).w = vehs_uv1(frmIDs(i), kk).w;
        vehicles_uv(kk).h = vehs_uv1(frmIDs(i), kk).h;
        vehicles_uv(kk).conf = vehs_uv1(frmIDs(i), kk).conf;
        vehicles_uv(kk).cls = vehs_uv1(frmIDs(i), kk).cls;
        vehicles_uv(kk).flag = vehs_uv1(frmIDs(i), kk).flag;
    end
    [idx_flag] = mbdselectObjsForDetail(vehicles_uv,[0 0 1920 1080], 0.15,50);

    % 处理障碍物检测结果
    [obstacles,obstacles_uv]  = mbdProcessObstacles(vehs_uv1(frmIDs(i), :),vehs_uv2(frmIDs(i), :),cropSizeObs(frmIDs(i), :),lanes_uv,lanes_xy);
    % 将新的障碍物检测结果输入障碍物追踪库
    % [eobstacles] = mbdAppendObstacles(obstacles,lstm_weights,lstm_mem_dict);
    [eobstacles] = mbdAppendObstacles(obstacles);

    %处理行人检测结果
    [pedstrains,pedstrains_uv] = mbdProcessPedstrains(peds_uv1(frmIDs(i),:),peds_uv2(frmIDs(i),:),cropSizeObs(frmIDs(i),:), obstacles_uv);
    % 将新的行人检测结果输入障碍物追踪库
    [eobstacles_Peds] = mbdAppendPedstrains(pedstrains);


    lanes_uv_2nd = auxLaneME2UV(lanes_me);
    lanes_uv_3rd = auxLaneEO2UV(lanes_eo.line);

end


%
% 源数据路径
% vid_path = 'F:\RawData\20210818_Cal\video';
% vidfiles = {...
%     '2021-08-18-10-13-20.mp4';...
%     '2021-08-18-10-19-50.mp4';...
%     '2021-08-18-10-28-17.mp4'...
%     };

% vidfile = vidfiles{1};

% 文件名
[~, name, ~] = fileparts(vidfile);

vid = VideoReader(fullfile(vid_path, vidfile));

frmID0 = int32(1);
% frmID1 = int32(floor(vid.Duration * vid.FrameRate));
frmID1 = int32(1);
frmIDs = frmID0:frmID1;

% 车辆
mbdInitializeEgo(g_ego_params.axle_nums, g_ego_params.width, ...
    g_ego_params.length, g_ego_params.height, g_ego_params.front_overhang, ...
    g_ego_params.front_wheelbase,g_ego_params.axle12_base, ...
    g_ego_params.axle23_base, g_ego_params.axle34_base, ...
    g_ego_params.total_mass, g_ego_params.curb_mass, ...
    g_ego_params.axle1_weight, g_ego_params.axle2_weight, ...
    g_ego_params.axle3_weight, g_ego_params.axle4_weight);

% 标定初始化
[u, v, w, h] = mbdInitializePSS((1824.94313), (1821.13332), (956.27022), (567.59414), uint32([1080, 1920]), [-10, 5], [-2, 2]);

% 逐帧处理车道线、障碍物结果
vid.CurrentTime = (double(frmIDs(1))-1)/vid.FrameRate;

for i=1:length(frmIDs)
    if ~hasFrame(vid), break; end

    mbdUpdateEgo(65, 0, 0, 0, 0, 0, 0, 0, 0, 0, ...
        wiper_status.no_action, indicator_status.no_action);

    frm = readFrame(vid);
    %     [~, uv, ~, lineL, lineR, status, progress1, progress2] = mbdUpdatePSS(frm(:,:,2));
    scale = CalibState.scale;
    img = imresize(frm,scale);
    [~, uv, result_raw,~, Line_PL,Line_PR, status, progress1, progress2]= mbdUpdatePSS_1(lanes_nn1(frmIDs(i)));

    if status == int8(1)
        break;
    end
end

[err, cali] = mbdTerminatePSS();

%%
mbdPreserveCamera();


