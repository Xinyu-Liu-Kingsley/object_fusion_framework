function [obsCIPV,obsL,obsR] = save_frame_VehCIPV(obsCIPV,obsL,obsR,frmidx)
% 获取保存主目标单帧检测信息，u v w h cls..


global  g_tracks_auto 

% 更新CIPV
if g_tracks_auto.idxCIPV
    obs_XY = g_tracks_auto.matrix(prev_cursor(g_tracks_auto.cursor, obstacle_const.track_depth), g_tracks_auto.idxCIPV);
    obsCIPV(frmidx,1) = obs_XY.x; %
    obsCIPV(frmidx,2) = obs_XY.y;
    obsCIPV(frmidx,3) = obs_XY.x1;
    obsCIPV(frmidx,4) = obs_XY.x2;
    obsCIPV(frmidx,5) = obs_XY.u;
    obsCIPV(frmidx,6) = obs_XY.v;
    obsCIPV(frmidx,7) = obs_XY.w;
    obsCIPV(frmidx,8) = obs_XY.h;
    obsCIPV(frmidx,9) = obs_XY.r;
    obsCIPV(frmidx,10) = obs_XY.cls;

    obsCIPV(frmidx,11) = obs_XY.x3;
    obsCIPV(frmidx,12) = obs_XY.obsW1;
    obsCIPV(frmidx,13) = obs_XY.direction;

end

minXL = 200; idxL = 255;
minXR = 200; idxR = 255;
for i = 1:1:obstacle_const.track_width
    if   ~g_tracks_auto.status(i).used
        continue;
    end
    obs_XY = g_tracks_auto.matrix(prev_cursor(g_tracks_auto.cursor, obstacle_const.track_depth), i);
    if obs_XY.flag && (obs_XY.x<minXL || obs_XY.x<minXR)

        % 左侧
        if obs_XY.laneIndex==uint8(2) &&(obs_XY.x<minXL)
            minXL = obs_XY.x;
            idxL = i;
        end
        % 右侧
        if obs_XY.laneIndex==uint8(3) && (obs_XY.x<minXR)
            minXR = obs_XY.x;
            idxR = i;
        end
    end
end
if idxL~=255
    obs_XY = g_tracks_auto.matrix(prev_cursor(g_tracks_auto.cursor, obstacle_const.track_depth), idxL);
    % [x,y,x1,x2,u,v,w,h,r,cls];
    obsL(frmidx,1) = obs_XY.x; % 
    obsL(frmidx,2) = obs_XY.y;
    obsL(frmidx,3) = obs_XY.x1;
    obsL(frmidx,4) = obs_XY.x2;
    obsL(frmidx,5) = obs_XY.u;
    obsL(frmidx,6) = obs_XY.v;
    obsL(frmidx,7) = obs_XY.w;
    obsL(frmidx,8) = obs_XY.h;
    obsL(frmidx,9) = obs_XY.r;
    obsL(frmidx,10) = obs_XY.cls;

    obsL(frmidx,11) = obs_XY.x3;
    obsL(frmidx,12) = obs_XY.obsW1;
    obsL(frmidx,13) = obs_XY.obsW2;
end
if idxR~=255
     obs_XY = g_tracks_auto.matrix(prev_cursor(g_tracks_auto.cursor, obstacle_const.track_depth), idxR);
    % [x,y,x1,x2,u,v,w,h,r,cls];
    obsR(frmidx,1) = obs_XY.x; % 
    obsR(frmidx,2) = obs_XY.y;
    obsR(frmidx,3) = obs_XY.x1;
    obsR(frmidx,4) = obs_XY.x2;
    obsR(frmidx,5) = obs_XY.u;
    obsR(frmidx,6) = obs_XY.v;
    obsR(frmidx,7) = obs_XY.w;
    obsR(frmidx,8) = obs_XY.h;
    obsR(frmidx,9) = obs_XY.r;
    obsR(frmidx,10) = obs_XY.cls;

    obsR(frmidx,11) = obs_XY.x3;
    obsR(frmidx,12) = obs_XY.obsW1;
    obsR(frmidx,13) = obs_XY.obsW2;
end

end