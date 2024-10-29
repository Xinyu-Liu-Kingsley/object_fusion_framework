function [tracks,isFill] = updateTracksO(tracks, cfg)    %#codegen
% 刷新障碍物轨迹库
% global g_ego_status g_tracks_lane;
%
% thre_yaw = 4;

% 更新处理标识位
for jdx=1:obstacle_const.track_width
    [tracks, ~] = update_state(tracks, cfg, jdx);% 更新跟踪状态
end

isFill = false(obstacle_const.max_obs_fuse,1);
for i=1:obstacle_const.track_width
    %     direction = tracks.output(i).obj_direction;
    tracks.output(i) = obstacle_const.FusedObject;

    % 卡尔曼滤波
    if tracks.status(i).used
        if tracks.status(i).flag
            obs_XY = tracks.matrix(tracks.cursor, i);
        else
            obs_XY = tracks.matrix(prev_cursor(tracks.cursor, obstacle_const.track_depth), i);
        end
        [obs_XY] = get_TTC(obs_XY); % 计算TTC
        [tracks, est] = kalman_obstacle_filter(tracks, cfg, obs_XY.x, obs_XY.y,obs_XY.velo_x,obs_XY.velo_y,obs_XY.ax,obs_XY.ay, i, 0);

        % 更新tracks.estimated
        tracks.estimated(i).x = est(1);
        tracks.estimated(i).y = est(2);
        tracks.estimated(i).velo_x = est(3);
        tracks.estimated(i).velo_y = est(4);
        tracks.estimated(i).state = tracks.status(i).state;

        % update y_history vx_history vy_history，判断目标的运动状态,仅在120m范围内判断运动状态
        if tracks.status(i).state>=track_status.stable && tracks.estimated(i).y<=120
            %             tracks = update_rel_status(tracks,i,direction);
            tracks = updateMotionStatus(tracks,i);
        end

        % 仅对有效范围内的目标轨迹做输出
        if  est(1)>obstacle_const.cfg.x_coverage(1) &&...
                est(1)<obstacle_const.cfg.x_coverage(2) &&...
                est(2)>obstacle_const.cfg.y_coverage(1) &&...
                est(2)<obstacle_const.cfg.y_coverage(2)*1.2
            tracks.output(i).x = est(1);
            tracks.output(i).fusedRadID = obs_XY.fusedRadID;
            tracks.output(i).fusedCamID = obs_XY.fusedCamID;
            tracks.output(i).y = est(2);
            % TEMP,延迟相对速度输出
            if tracks.status(i).CNT>50
                tracks.output(i).velo_x = tracks.estimated(i).velo_x;
            else
                tracks.output(i).velo_x = 0;
            end
            tracks.output(i).velo_y = est(4);
            tracks.output(i).state = tracks.status(i).state;
            tracks.output(i).ay = obs_XY.ay;
            tracks.output(i).cls = obs_XY.cls;
            tracks.output(i).obj_width = obs_XY.obj_width;
            tracks.output(i).obj_det_prop = obs_XY.obj_det_prop;
            %             tracks.output(i).obj_exist_prop =  obs_XY.obj_exist_prop;
%             if tracks.output(i).det_src == det_src.front
            tracks.output(i).obj_exist_prop =  obj_exist_prop.percent_99;
%             else
%                 tracks.output(i).obj_exist_prop =  obs_XY.obj_exist_prop;
%             end
            tracks.output(i).obj_dyn_prop = obs_XY.obj_dyn_prop;
            tracks.output(i).obj_lane =  obs_XY.obj_lane;
            tracks.output(i).obj_direction = obs_XY.obj_direction;
            tracks.output(i).img_lane = obs_XY.img_lane;
            tracks.output(i).moveType = obs_XY.moveType;  % 雷达输出目标类型
            tracks.output(i).obsolute_res = obs_XY.obsolute_res;  % 雷达输出目标运动属性
            tracks.output(i).det_src = obs_XY.det_src;
            tracks.output(i).TTC = obs_XY.TTC;
            isFill(i) = true;
        end
    end
end
% idxCIPVlast = tracks.idxCIPV ;
tracks.idxCIPV = uint8(0);
[tracks,idx] = update_CIPV(tracks);
% if idx~= idxCIPVlast && tracks.CNT>= obstacle_const.cfg.thres_IDassign  && idx~=0
%   tracks.output(idx).obj_width = 1000;
% end
tracks.idxCIPV = uint8(idx);

% tracks.estimated update:
% if  idx~=uint8(0)
%     tracks.estimated(idx).CIPP = tracks.output(idx).CIPV;
%     tracks.estimated(idx).CIPP = tracks.output(idx).CIPP;
%     tracks.estimated(idx).TTC = tracks.output(idx).TTC;
%     tracks.estimated(idx).THW = tracks.output(idx).THW;
% end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 为保证状态的一致，flag的复位与cursor的移动需相邻、成对操作！！！
% 复位处理标识
for i=1:obstacle_const.track_width
    tracks.status(i).flag = uint8(0);
end
% 车道线轨迹库游标向前移动一位
tracks.cursor = ...
    next_cursor(tracks.cursor, obstacle_const.track_depth);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 生命计数器增一
tracks.CNT = tracks.CNT + int32(1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end


function [tracks,idx] = update_CIPV(tracks)
% 更新CIPV 和 CIPP
% 清0
obstacles_eo = tracks.output;
for i=1:obstacle_const.track_width
    obstacles_eo(i).CIPV = uint8(0);
    obstacles_eo(i).CIPP = uint8(0);
end

nearest = 500;
idx = uint8(0);

for i=1:obstacle_const.track_width
    if obstacles_eo(i).state < track_status.stable
        continue;
    end
    CIPVFlag = uint8(0);
    fusedState = ((obstacles_eo(i).obj_det_prop==obs_det_prop.sole_radar||obstacles_eo(i).obj_det_prop==obs_det_prop.sole_camera)&&... %tracks.status(i).CNTFused>=obstacle_const.cfg.thres_IDassign)...
        tracks.status(i).CNTFused>=obstacle_const.cfg.thres_IDassign)...
        ||obstacles_eo(i).obj_det_prop==obs_det_prop.fused;   % 处于融合状态，或者处于单Radar状态，但之前有融合状态且融合状态
    if obstacles_eo(i).obj_det_prop==obs_det_prop.sole_radar && ...  % 给之前有融合状态，后期摄像头目标丢失的融合目标类型赋值，cls值
            tracks.status(i).CNTFused>=obstacle_const.cfg.thres_IDassign
        obstacles_eo(i).cls = tracks.status.fusedCls;
    end
    if (obstacles_eo(i).obj_lane == obj_lane.sameLane || obstacles_eo(i).img_lane == obj_lane.sameLane) && fusedState
        CIPVFlag = uint8(1);
    end

    if (obstacles_eo(i).state >= track_status.stable) && (obstacles_eo(i).y < nearest) && CIPVFlag == uint8(1)
        nearest = obstacles_eo(i).y;
        idx = i;
    end
end

if idx ~= uint8(0)
    if obstacles_eo(idx).cls == obstacle_cls.car || ...
            obstacles_eo(idx).cls == obstacle_cls.truck || ...
            obstacles_eo(idx).cls == obstacle_cls.bus ||(obstacles_eo(idx).obj_det_prop==obs_det_prop.sole_radar) % 避免雷达没有目标类型
        obstacles_eo(idx).CIPV = uint8(1);
    elseif obstacles_eo(idx).cls == obstacle_cls.pedestrian || ... % 行人和自行车设置成CIPP
            obstacles_eo(idx).cls ==obstacle_cls.bicycle
        obstacles_eo(idx).CIPP = uint8(1);
    end
end
tracks.output = obstacles_eo;
% 根据车道线判断逻辑去除
% else
%     yl = g_cfg_lane.def_wid_half;
%     yr = -g_cfg_lane.def_wid_half;
%     for i=1:obstacle_const.track_width
%         if (obstacles_eo(i).state < track_status.stable)
%             continue;
%         end
%         if (obstacles_eo(i).state >= track_status.stable) && (obstacles_eo(i).y < yl) && (obstacles_eo(i).y > yr) && (obstacles_eo(i).x < nearest)
%             nearest = obstacles_eo(i).x;
%             idx = i;
%         end
%     end
end




function [tracks, state] = update_state(tracks, g_cfg_auto, jdx)

coder.inline('always')
assert(isscalar(jdx))
assert(jdx >= 1 && jdx <= obstacle_const.track_width)

% global g_cfg_auto g_tracks_auto;
state = track_status.untracking;
if tracks.status(jdx).CNT > int32(0)
    if tracks.status(jdx).CNT < g_cfg_auto.thres_stable
        if tracks.status(jdx).CNT < obstacle_const.len_1st
            state = track_status.initial;
        else
            state = track_status.growing;
        end
    elseif tracks.status(jdx).LST >= g_cfg_auto.thres_unstable
        state = track_status.unstable;
    else
        state = track_status.stable;
    end
end
tracks.status(jdx).state = state;

end



function  tracks = update_rel_status(tracks,i,direction)
coder.inline('always');
global g_ego_status;
maxTargetVx = 150; % kph

y_history = tracks.status(i).y_history;
tracks.status(i).y_history(end) = tracks.estimated(i).y;
tracks.status(i).y_history(1:end-1) = y_history(2:end);
deltaD = tracks.status(i).y_history(end)-tracks.status(i).y_history(1);
frmCNT = 10;
moveD = mean(g_ego_status.vx_history)*(1/double(cam_const.FPS))*frmCNT;

maxCNT = 15;
cntThres = 10;
% 假定为同向
if deltaD<0 % 目标车速小于自车速
    if abs(deltaD+moveD)<2  % 目标车静止
        rel_status = uint8(1);
    else
        rel_status = uint8(2);
    end
else  % 目标车速大于等于自车速
    rel_status = uint8(3);
end

if  tracks.status(i).rel_status(2)==rel_status || tracks.status(i).rel_status(2)==uint8(0)
    tracks.status(i).rel_status(2) = rel_status;
    tracks.status(i).rel_status(1) = min(tracks.status(i).rel_status(1)+uint8(1),maxCNT);
else

    if tracks.status(i).rel_status(1)>cntThres
        tracks.status(i).rel_status(1) = tracks.status(i).rel_status(1)-uint8(1);
    else
        tracks.status(i).rel_status(2) = rel_status;
        tracks.status(i).rel_status(1) = uint8(1);
    end
end

% 使用rel_status
% if tracks.status(i).rel_status(2)==uint8(1) &&  tracks.status(i).rel_status(1)>cntThres
%     tracks.estimated(i).velo_y = -g_ego_status.vx;
% end

% if  direction == uint8(1) % 同向
%     if tracks.status(i).rel_status(2)==uint8(2) &&  tracks.status(i).rel_status(1)>cntThres  %rel_vx<0
%         tracks.estimated(i).rel_vx = max(tracks.estimated(i).rel_vx,-g_ego_status.vx);
%
%     elseif tracks.status(i).rel_status(2)==uint8(3) &&  tracks.status(i).rel_status(1)>cntThres %rel_vx>0
%         tracks.estimated(i).rel_vx = min(tracks.estimated(i).rel_vx,maxTargetVx/3.6-g_ego_status.vx);
%     end
% end

end
