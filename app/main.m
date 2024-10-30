% Description: 融合demo，与capsule保持一致；
% log:
% --Camera ObjectList
% --Radar  ObjecetList
%*************************************************************************%
close all
clear
clc
% 源数据路径(需更新)
[raw_path,dat_path,vid_path] = get_local_path_const(); 
files = dir(fullfile(vid_path, '*.avi')); %% 根据视频后缀修改'*.avi'
videoNo = 1;               % 视频序号，258:289
frmID_start = 1;           % 回放数据起始帧，结束帧在initialVideo赋值
%% ================ 请确认是否记录保存 ======================================== %%
displayFlag = 1;                            % 是否显示画图
saveVideo = 0;                              % 是否保存回放视频，1：保存
saveCipvRadar = 0;                          % 是否保存雷达主目标
readVideo = 0;                              % 是否有录屏的视频文件
saveVideo = saveVideo&displayFlag;
displayMDCU = 0;                            % 是否显示MDCU结果
versionNum = 1018;
%% ================= 初始化参数 =============================================== %%
vidfile = files(videoNo(1)).name;
vid = VideoReader(fullfile(vid_path, vidfile));
% 获取文件名
[~, name, ~] = fileparts(vidfile);
% 创建全局变量
create_global_vars;
global g_ego_params  g_cfg_auto;
% 配置显示信息
mbdCfgDispInfo(info_cls.anything, info_spec.warning);
% 车辆
mbdInitializeEgo(g_ego_params.axle_nums, g_ego_params.width, ...
    g_ego_params.length, g_ego_params.height, g_ego_params.front_overhang, ...
    g_ego_params.front_wheelbase,g_ego_params.axle12_base, ...
    g_ego_params.axle23_base, g_ego_params.axle34_base, ...
    g_ego_params.total_mass, g_ego_params.curb_mass, ...
    g_ego_params.axle1_weight, g_ego_params.axle2_weight, ...
    g_ego_params.axle3_weight, g_ego_params.axle4_weight);
% 障碍物vid.FrameRate
mbdInitizlizeObstacle(vid.FrameRate, g_cfg_auto.min_conf, ...
    g_cfg_auto.x_coverage, g_cfg_auto.y_coverage, ...
    g_cfg_auto.ref_widths, g_cfg_auto.thres_stable, ...
    g_cfg_auto.thres_unstable, g_cfg_auto.thres_lost);

% !!!更新自车车身参数 !!!!!
g_ego_params.width = 2.4;             % m, 车身宽度；
g_ego_params.front_overhang = 1.45;   % m, 前悬长度；
g_ego_params.length = 6;              % m, 车长；

%% ==============  获取雷达和摄像头数据============================================ %%
% 暂时删除该数据，未测试 testData\CameraMatrix.mat RadMatrix.mat
% getRadarCameraData;
% frameData = frmData;

%% =============== 循环视频融合模块 =============================================== %%
for x= videoNo
    % -------------------视频及数据读取初始化---------------------------------------- %
    initialVideo;
    mbdInitialGlobalVars();
%     [g_tracks_auto,g_tracks_ego] = mbdInitialGlobalVars(obstacle_const,ego_const);
    % ------------------- 循环帧融合模块 -------------------------------------------- %
    for i=1:length(frmIDs)
        if ~hasFrame(vid), break; end
        frm = readFrame(vid);
        % 更新车速
        mbdUpdateEgo(carSignals(frmIDs(i),1), 0, 0, 0, 0, carSignals(frmIDs(i),2), carSignals(frmIDs(i),3), 0, 0, 0, ... %carSignalArray(frmIDs(i),1)
            wiper_status.no_action, indicator_status.no_action);
        % Radar数据预处理
        frameDataSingle = mbdProcessFrame(frameData(frmIDs(i)));
        % 更新融合障碍物
        eobstacles = mbdUpdateFusion(frameDataSingle);

        % ---------------- 动态显示 -------------------------------------------------- %
        dynamicDisplayFusion;
        pause(0.0001)

        % ---------------- 存视频 ---------------------------------------------------- %
        if saveVideo
            try
                writeVideo(v_res, getframe(h));
            catch
                continue
            end
        end

    end

    % 画出cipv轨迹
        dispAndSaveFigure;
    %     mbdPreserveCamera();
    if saveVideo
        close(v_res); 
    end
end
