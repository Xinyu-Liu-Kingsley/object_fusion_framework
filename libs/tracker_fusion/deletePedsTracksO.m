function tracks = deletePedsTracksO(tracks, g_cfg_auto)    %#codegen
% 删除丢失轨迹

% global g_tracks_auto g_cfg_auto;

for jdx=1:peds_const.track_width
    % 如果目标丢失次数达到上限则丢弃该轨迹
    if tracks.status(jdx).used == uint8(1) && ((tracks.status(jdx).LST >= g_cfg_auto.thres_lost) || (tracks.status(jdx).CNT < tracks.status(jdx).LST))
        tracks = resetPedsTrackO(tracks, jdx);

        % bbox tracking
        tracks.uvTracker(jdx) = peds_const.uvTracker;
    end
end

end

