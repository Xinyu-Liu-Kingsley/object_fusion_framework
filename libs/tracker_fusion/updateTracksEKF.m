function [tracks] = updateTracksEKF(tracks, cfg,isAuto)    %#codegen


% 刷新障碍物轨迹库
global g_tracks_auto g_cfg_auto g_cam_params;
% EKF模型处理
% 未匹配成功预测；
% 长时间丢失轨迹删除；
% 创建新轨迹；
% 被合并
% 更新预测

%
[flag_tracks] = get_flag_tracks (tracks);
% 无目标退出
if sum(flag_tracks)<1
    return
end

%获取当前帧输入
% 获取当前帧模型输入值
[tracks] = input_EKF(tracks,flag_tracks);
% 初始化协方差矩阵
% 初始化新创建lstm long short
[tracks] = initial_EFK(tracks,flag_tracks);
% 长时间丢失轨迹
[tracks.ekfOutput] = delete_EFK(tracks.ekfOutput,flag_tracks);
% Ekf滤波
[tracks.ekfOutput] = update_obstacle(tracks.ekfOutput);

end

function [tracks] = input_EKF(tracks,flag_tracks)
% 获取当前帧EKF模型的输入值

if tracks.cursor == 1
    cursor_last = obstacle_const.track_depth;
else
    cursor_last = tracks.cursor-uint8(1);
end
for  i = 1: obstacle_const.track_width
    switch flag_tracks(i)
        case 1
            tracks.ekfOutput(i).w = single(tracks.matrix(tracks.cursor, i).w); % bbox宽度
            tracks.ekfOutput(i).x_width =  single(obstacle_const.cfg.ref_widths(tracks.matrix(tracks.cursor, i).cls));
            tracks.ekfOutput(i).delta_w = 0;
            tracks.ekfOutput(i).flag = uint8(1);
            tracks.ekfOutput(i).P = single([100 0;0 0.01]);
        case 2
            w = tracks.matrix(tracks.cursor, i).w;
            w_last = tracks.matrix(cursor_last, i).w;
            tracks.ekfOutput(i).w = single(tracks.matrix(tracks.cursor, i).w); % bbox宽度
            tracks.ekfOutput(i).x_width =  single(obstacle_const.cfg.ref_widths(tracks.matrix(tracks.cursor, i).cls));%目标宽度
            tracks.ekfOutput(i).delta_w = abs(w_last-w);%前后帧宽度变化
            tracks.ekfOutput(i).flag = uint8(1);
        case 3
        case 4
            w = tracks.matrix(tracks.cursor, i).w;
            w_last = tracks.matrix(cursor_last, i).w;
            tracks.ekfOutput(i).w = single(tracks.matrix(tracks.cursor, i).w);  % bbox宽度
            tracks.ekfOutput(i).x_width =  single(obstacle_const.cfg.ref_widths(tracks.matrix(tracks.cursor, i).cls));%目标宽度
            tracks.ekfOutput(i).delta_w = abs(w_last-w);%前后帧宽度变化
            tracks.ekfOutput(i).flag = uint8(1);
    end
end
end


function [ekfOutput] = delete_EFK(ekfOutput,flag_tracks)
% 清除无效
for  i = 1: obstacle_const.track_width
    if ekfOutput(i).flag && flag_tracks(i)==uint8(0)
        ekfOutput(i) = obstacle_const.ekfRecord;
    end
end
end


function  [tracks] = initial_EFK(tracks,flag_tracks)
% 新创建 初始化 P
for  i = 1: obstacle_const.track_width
    if flag_tracks(i) == uint8(1)
        tracks.ekfOutput(i).P  = single([100 0;0 0.01]);
        tracks.ekfOutput(i).X = single([tracks.matrix(tracks.cursor, i).x2;0]);
    end
end
end

function  [ekfOutput] = update_obstacle(ekfOutput)
% 对有效轨迹
% 推理
global  g_cam_params g_cfg_auto;
dt = g_cfg_auto.deltaT;
A = [1 1;0 1];
Q = [0.0001 0;0 0.0001];
R= 4;
for  i = 1: obstacle_const.track_width
    if ekfOutput(i).flag
        x_t = ekfOutput(i).X;
        P_t = ekfOutput(i).P;
        veh_w3d = ekfOutput(i).x_width;
        bbox_w = ekfOutput(i).w;
        delta_w = ekfOutput(i).delta_w;
        [x_t,P_t] = extend_kalman_obstacle_filter(x_t,P_t,A,Q,R,g_cam_params.fx,veh_w3d,bbox_w,delta_w);
        ekfOutput(i).X = x_t;
        ekfOutput(i).P = P_t;
        ekfOutput(i).x = x_t(1);
        ekfOutput(i).vx = x_t(2)*15;
    end
end
end


function [x_t, P_t]= extend_kalman_obstacle_filter(x_t,P_t,A,Q,R,fx,veh_w3d,w2d,delta_w)
%预测
x_t_ = A*x_t;
P_t_ = A*P_t*A'+Q;

zt =w2d;
zth = fx*veh_w3d/(x_t_(1)+1e-6);

%估计
H = [-fx*veh_w3d/(x_t_(1)*x_t_(1)+1e-6) 0];
if delta_w>5
    S_t = H*P_t_'*H'+1e5;
else
    S_t = H*P_t_'*H'+R;
end
%     S_t_ = inv(S_t);
klm_gain = P_t_*H'/S_t; %卡尔曼增益
x_t =  x_t_+klm_gain*(zt-zth);
P_t = P_t_-klm_gain*H*P_t_;
end







