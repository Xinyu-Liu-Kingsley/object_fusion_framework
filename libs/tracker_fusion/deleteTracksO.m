function tracks = deleteTracksO(tracks, g_cfg_auto)    %#codegen
% 删除丢失轨迹

% global g_tracks_auto g_cfg_auto;

for jdx=1:obstacle_const.track_width
    % 如果目标丢失次数达到上限则丢弃该轨迹
    if tracks.status(jdx).used == uint8(1) && ((tracks.status(jdx).LST >= g_cfg_auto.thres_lost) || (tracks.status(jdx).CNT < tracks.status(jdx).LST))
        tracks = resetTrackO(tracks, jdx);
    end
%     % 没有Camera数据，雷达连续丢帧，删除航迹
%     if tracks.status(jdx).used == uint8(1) && tracks.status(jdx).LSTRad >= g_cfg_auto.thres_lost
%       tracks = resetTrackO(tracks, jdx);
%     end
%     % Radar航迹创建稳定，大于稳定该阈值(30帧)，但Camera没有检测到，删除航迹,适配切入，切出，鬼探头情况
%     if tracks.status(jdx).used == uint8(1) && tracks.status(jdx).CNTRad >= g_cfg_auto.thres_stable*5 && tracks.status(jdx).LSTCam >= g_cfg_auto.thres_lost% 认为雷达比摄像头创建航迹30帧
%       tracks = resetTrackO(tracks, jdx);
%     end
end

end

