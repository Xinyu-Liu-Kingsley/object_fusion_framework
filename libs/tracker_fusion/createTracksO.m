function tracks = createTracksO(tracks, cfg, obs_XYs, flags)    %#codegen
% 创建新轨迹

% global tracks;

for i=1:obstacle_const.max_obs_fuse
    if flags(i) == uint8(0) && obs_XYs(i).flag == uint8(1)
        flag = uint8(0);   % 数据点处理标识
        for jdx=1:obstacle_const.track_width
            if tracks.status(jdx).used == uint8(0)
                [tracks] = fullInNewTracks(tracks,jdx,obs_XYs(i),cfg);
                flag = uint8(1);
                break;
            end
        end
        if flag == uint8(0)
            [iTracks,tracks] = deleteForNewTracks(tracks,obs_XYs(i));
            if iTracks ~= uint8(255)
                [tracks] = fullInNewTracks(tracks,iTracks,obs_XYs(i),cfg);
            else  %  若未被处理，则打印信息提示
                disp_info(sprintf('createObstacleTrack >> new track to mat failed! C : [%+6.2f, %+6.2f]', ...
                    obs_XYs(i).x, obs_XYs(i).y), info_cls.obstacle, info_spec.warning)
%                 message = sprintf('createObstacleTrack >> new track to mat failed! C : [%+6.2f, %+6.2f]', obs_XYs.x, obs_XYs.y);
%                 fprintf('%s [%s] [%s]\n', message, info_cls.obstacle, info_spec.warning);
            end
        end
    end
end

end
function [iTracks,tracks] = deleteForNewTracks(tracks,obs_XYs)
% 轨迹空间已满，删除横向纵向最远目标，用于存放新的轨迹；
% 第i个obs_XYs

maxY = 2.5;
maxX = 0;
y = 0;
iTracks = uint8(255);
for idx=1:obstacle_const.track_width
    if tracks.status(idx).flag
        trackXY = tracks.matrix(tracks.cursor, idx);
    else
        trackXY = tracks.matrix(prev_cursor(...
            tracks.cursor, obstacle_const.track_depth), idx);
    end
    obs_state = ~(tracks.output(idx).obj_det_prop == obs_det_prop.sole_camera ||...
        tracks.output(idx).obj_det_prop == obs_det_prop.fused) ;
    if (abs(trackXY.x)) > maxX && (abs(trackXY.y)>maxY) && obs_state
        maxX = abs(trackXY.x);
        maxY = abs(trackXY.y);
        y = trackXY.y;
        iTracks = idx;
    end
end
if (abs(obs_XYs.x) < maxX || abs(obs_XYs.y)<maxY) && iTracks~= uint8(255) % 若轨迹被删除处理，则打印信息提示
    disp_info(sprintf('deleteForNewTracks >> track is deleted! C : [%+6.2f, %+6.2f]', ...
        maxX, y), info_cls.obstacle, info_spec.warning)
    tracks = resetTrackO(tracks, iTracks);
else
    iTracks = uint8(255);
end

end

function  [tracks] = fullInNewTracks(tracks,jdx,obs_XYs,cfg)
% 在空的轨迹库中填充数据

% 对于初始创建tracks，测距>100时，x值优先选用宽度测距结果；
%     if obs_XYs.x > 100
%         obs_XYs.x = obs_XYs.x2;
%     end

tracks.matrix(tracks.cursor, jdx) = obs_XYs;
[tracks.status(jdx).CNT, tracks.status(jdx).LST] = ...
    inc_frame(tracks.status(jdx).CNT, tracks.status(jdx).LST);
% 更新RadCNT
if obs_XYs.fusedRadID > uint8(0)
    [tracks.status(jdx).CNTRad, tracks.status(jdx).LSTRad] = ...
        inc_frame(tracks.status(jdx).CNTRad, tracks.status(jdx).LSTRad);
end
% 更新CamID
if obs_XYs.fusedCamID > uint8(0)
    [tracks.status(jdx).CNTCam, tracks.status(jdx).LSTCam] = ...
        inc_frame(tracks.status(jdx).CNTCam, tracks.status(jdx).LSTCam);
end


tracks.status(jdx).CNT = int32(1);
tracks.status(jdx).CNTCam = int32(1);
tracks.status(jdx).CNTRad = int32(1);
tracks.status(jdx).used = uint8(1);
tracks.status(jdx).flag = uint8(1);
tracks.status(jdx).start_Range = obs_XYs.y;
%     tracks.status(jdx).cls = obs_XYs.cls;

% 滤波参数初始化
[tracks, ~] = kalman_obstacle_filter(tracks, cfg, obs_XYs.x, obs_XYs.y,obs_XYs.velo_x,obs_XYs.velo_y,obs_XYs.ax,obs_XYs.ay, jdx, 1);

end