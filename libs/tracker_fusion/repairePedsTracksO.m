function tracks = repairePedsTracksO(tracks)    %#codegen
% 针对未匹配的轨迹进行维护
% 同时维护目标框跟踪轨迹，以预测代替当前值进行更新；
% global g_tracks_auto;

% 更新历史轨迹状态
for jdx=1:peds_const.track_width
    % 如果该条车道线跟踪轨迹被占用，但是本轮没有更新
    if tracks.status(jdx).flag == uint8(0) && ...
            tracks.status(jdx).used == uint8(1)
        [tracks.status(jdx).CNT, tracks.status(jdx).LST] = ...
            dec_frame(tracks.status(jdx).CNT, tracks.status(jdx).LST);
        tracks.matrix(tracks.cursor, jdx) = ...
            tracks.matrix(prev_cursor(tracks.cursor, peds_const.track_depth), jdx); % 使用前一帧进行更新
        tracks.lstflg(tracks.cursor, jdx) = uint8(1);
        tracks.status(jdx).flag = uint8(1);  % 更新处理标识

        % bbox tracking
        stateFlag = peds_const.uvTrackerStatus.pred;
        uv = single([0; 0; 0; 0]);
        tracks.uvTracker(jdx).uv = uv;
        [tracks.uvTracker(jdx).X,tracks.uvTracker(jdx).P,tracks.uvTracker(jdx).predZ] = kalmanFilterPixelO(uv, ...
            tracks.uvTracker(jdx).X,tracks.uvTracker(jdx).P,stateFlag);

        
    end
end

end
