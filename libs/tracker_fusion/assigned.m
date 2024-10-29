function [g_tracks_auto, obs_XY]= assigned(g_tracks_auto, jdx, obs_XY)
% 更新关联上的轨迹；
% 向障碍物轨迹添加新的数据点；

coder.inline('always')
assert(obs_XY.flag ~= uint8(0))

% 添加数据点
g_tracks_auto.matrix(g_tracks_auto.cursor, jdx) = obs_XY;
g_tracks_auto.lstflg(g_tracks_auto.cursor, jdx) = uint8(0);
% [g_tracks_auto, obs_XY] = clsConfirm2fixDist(g_tracks_auto, jdx, obs_XY);

% 更新计数器,更新Camera关联
if obs_XY.fusedCamID > uint8(0)
    [g_tracks_auto.status(jdx).CNTCam, g_tracks_auto.status(jdx).LSTCam] = ...
        inc_frame(g_tracks_auto.status(jdx).CNTCam, g_tracks_auto.status(jdx).LSTCam);
else
    [g_tracks_auto.status(jdx).CNTCam, g_tracks_auto.status(jdx).LSTCam] = ...
        dec_frame(g_tracks_auto.status(jdx).CNTCam, g_tracks_auto.status(jdx).LSTCam);
end
% 更新计数器，更新Radar关联 
% 如果radarID 发生跳变，则CNTRad需要清零，避免出现误关联，雷达和摄像头稳定关联帧数要大于20帧
cnt = g_tracks_auto.status(jdx).CNT;
if cnt <=10
    RadID = g_tracks_auto.status(jdx).RadID_history(cnt);
else
    RadID = g_tracks_auto.status(jdx).RadID_history(10);
end
% 如果关联的ID切换，则Radar的计数清0,同时关联误差也要清零
if RadID ~= obs_XY.fusedRadID && RadID>uint8(0)
    g_tracks_auto.status(jdx).CNTRad = int32(0);
    g_tracks_auto.status(jdx).LSTRad = int32(0);
    g_tracks_auto.status(jdx).x_error_history = zeros(6,1);
    g_tracks_auto.status(jdx).y_error_history = zeros(6,1);
end
if obs_XY.fusedRadID > uint8(0)
    [g_tracks_auto.status(jdx).CNTRad, g_tracks_auto.status(jdx).LSTRad] = ...
        inc_frame(g_tracks_auto.status(jdx).CNTRad, g_tracks_auto.status(jdx).LSTRad);
else
    [g_tracks_auto.status(jdx).CNTRad, g_tracks_auto.status(jdx).LSTRad] = ...
        dec_frame(g_tracks_auto.status(jdx).CNTRad, g_tracks_auto.status(jdx).LSTRad);
end
% 更新融合状态计数
if obs_XY.fusedRadID > uint8(0) &&  obs_XY.fusedCamID > uint8(0)
     [g_tracks_auto.status(jdx).CNTFused, ~] = ...
        inc_frame(g_tracks_auto.status(jdx).CNTFused, int32(0));
end
if g_tracks_auto.status(jdx).CNTFused >= obstacle_const.cfg.thres_IDassign && obs_XY.fusedCamID > uint8(0)
    g_tracks_auto.status(jdx).fusedCls = obs_XY.cls;   % 避免单雷达没有目标类型
end

[g_tracks_auto.status(jdx).CNT, g_tracks_auto.status(jdx).LST] = ...
    inc_frame(g_tracks_auto.status(jdx).CNT, g_tracks_auto.status(jdx).LST);

% 更新tracks under the world标识位
g_tracks_auto.status(jdx).flag = uint8(1);

% 更新Assigned的CameraID和RadarID
[g_tracks_auto.status(jdx).CamID_history,g_tracks_auto.status(jdx).RadID_history] = updateAssignedID(g_tracks_auto,obs_XY,jdx);

% 更新Assigned的横纵向误差
[g_tracks_auto.status(jdx).x_error_history,g_tracks_auto.status(jdx).y_error_history] = updateAssignedError(g_tracks_auto,obs_XY,jdx);

% 判断关联误差是否增大
[g_tracks_auto.status(jdx).matchErrorFlag] = matchErrorJudge(g_tracks_auto,jdx);
end

