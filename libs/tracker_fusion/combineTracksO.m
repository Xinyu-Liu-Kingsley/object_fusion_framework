function tracks = combineTracksO(tracks, g_cfg_auto)    %#codegen
% 合并轨迹
% 处理大车目标分裂问题
% 处理之前未关联，但后来两条航迹关联的轨迹

persistent cnt cost ;
% costW;

% global g_tracks_auto g_cfg_auto;

if isempty(cnt)
    cnt = 0;
    cost = obstacle_const.defNominalErr * ones(obstacle_const.track_width);
    %     costW = zeros(obstacle_const.track_width,'uint8');
    disp_info('create persistent cnt in combineOTracks!', info_cls.obstacle)
end

if cnt == 0
    for idx=1:(obstacle_const.track_width-1)
        if tracks.status(idx).used
            obs_i = currentObsInMat(tracks,idx);
            camID_i = obs_i.fusedCamID;
            radID_i = obs_i.fusedRadID;
            det_src_i = obs_i.det_src;
            for jdx=(idx+1):obstacle_const.track_width
                if tracks.status(jdx).used && idx ~= jdx
                    obs_j = currentObsInMat(tracks,jdx);
                    camID_j = obs_j.fusedCamID;
                    radID_j = obs_j.fusedRadID;
                    det_src_j = obs_j.det_src;
                    if (camID_i == camID_j && camID_j ~= uint8(0)) || (radID_j == radID_i && radID_i ~=uint8(0)...
                            && det_src_i == det_src_j)
                        cost(idx, jdx) = 1;
                    end
                end
            end
        end
    end
end

cnt = cnt + 1;
if cnt >= obstacle_const.track_width
    disp_info('combineOTracks error!!!', info_cls.obstacle, info_spec.error)
    cnt = 0;
    return;
end

[M, I] = min(cost(:));

if M < obstacle_const.maxNominalErr*1000 %&& (costW(I)== uint8(3))    % 最小名义距离需满足预定阈值,目标框相差小于阈值像素
    [I_row, I_col] = ind2sub([obstacle_const.track_width, obstacle_const.track_width], I);

    % 通过CNT判定需要保留的轨迹kdx，以及相应的需删除的轨迹ddx
    if tracks.status(I_row).CNT > tracks.status(I_col).CNT
        kdx = int32(I_row);     % 保留的轨迹
        ddx = int32(I_col);     % 删除的轨迹
    else
        kdx = int32(I_col);     % 保留的轨迹
        ddx = int32(I_row);     % 删除的轨迹
    end
    Ix = [tracks.cursor:-1:1, obstacle_const.track_depth:-1:tracks.cursor+1];
    for i=1:obstacle_const.len_1st
        xdx = Ix(i);
        if tracks.lstflg(xdx, kdx) == uint8(1) && tracks.lstflg(xdx, ddx) == uint8(0)
            tracks.matrix(xdx, kdx) = tracks.matrix(xdx, ddx);
            tracks.lstflg(xdx, kdx) = uint8(0);
            if i == 1
                % 更新计数器 --补偿当前帧减掉的
                [tracks.status(kdx).CNT, tracks.status(kdx).LST] = ...
                    inc_frame(tracks.status(kdx).CNT, tracks.status(kdx).LST);
            end
            % 更新计数器--更新加1
            [tracks.status(kdx).CNT, tracks.status(kdx).LST] = ...
                inc_frame(tracks.status(kdx).CNT, tracks.status(kdx).LST);
        end
    end


    cost(I_row, I_col) = obstacle_const.defNominalErr;
    %     costW(I_row, I_col) = single(obstacle_const.defNominalErr);

    tracks = resetTrackO(tracks, ddx);
    tracks = combineTracksO(tracks, g_cfg_auto);
else
    cnt = 0;
    cost = obstacle_const.defNominalErr * ones(obstacle_const.track_width);
    %     costW = zeros(obstacle_const.track_width,'uint8');
end


end

