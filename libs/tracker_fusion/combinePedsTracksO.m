function tracks = combinePedsTracksO(tracks, g_cfg_auto)    %#codegen
% 合并轨迹

persistent cnt cost costW;

% global g_tracks_auto g_cfg_auto;

if isempty(cnt)
    cnt = 0;
    cost = peds_const.defNominalErr * ones(peds_const.track_width);
    costW = zeros(peds_const.track_width,'uint8');
    disp_info('create persistent cnt in combineOTracks!', info_cls.obstacle)
end

if cnt == 0
    for idx=1:(peds_const.track_width-1)
        if tracks.status(idx).used
            obs_i = currentObsInMat(tracks, idx, 0);%0.2;%
            s_x = interp1(g_cfg_auto.x, g_cfg_auto.s_x, obs_i.x, 'linear', 0.2);
            s_y = interp1(g_cfg_auto.x, g_cfg_auto.s_y, obs_i.x, 'linear', 0.2);
            for jdx=(idx+1):peds_const.track_width
                if tracks.status(jdx).used && idx ~= jdx
                    obs_j = currentObsInMat(tracks, jdx, 0);
                    cost(idx, jdx) = abs(obs_i.x - obs_j.x) * s_x + abs(obs_i.y - obs_j.y) * s_y;
                    % 增加两目标框的宽高及左上角横坐标限制
                    costW(idx, jdx) = uint8(abs(obs_i.w-obs_j.w)<= max(obs_i.w,obs_j.w)*0.2) +...
                        uint8(abs(obs_i.u-obs_j.u)< max(obs_i.w,obs_j.w)*0.25)+...
                        uint8(abs(obs_i.h-obs_j.h)< max(obs_i.h,obs_j.h)*0.2);
                end
            end
        end
    end
end

cnt = cnt + 1;
if cnt >= peds_const.track_width
    disp_info('combineOTracks error!!!', info_cls.obstacle, info_spec.error)
    cnt = 0;
    return;
end

[M, I] = min(cost(:));

if M < peds_const.maxNominalErr && (costW(I)== uint8(3))    % 最小名义距离需满足预定阈值,目标框相差小于阈值像素  
    [I_row, I_col] = ind2sub([peds_const.track_width, peds_const.track_width], I);
    
    % 通过CNT判定需要保留的轨迹kdx，以及相应的需删除的轨迹ddx
    if tracks.status(I_row).CNT > tracks.status(I_col).CNT
        kdx = int32(I_row);     % 保留的轨迹
        ddx = int32(I_col);     % 删除的轨迹
    else
        kdx = int32(I_col);     % 保留的轨迹
        ddx = int32(I_row);     % 删除的轨迹
    end
    Ix = [tracks.cursor:-1:1, peds_const.track_depth:-1:tracks.cursor+1];
    for i=1:peds_const.len_1st
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
    % bbox tracking
    if ddx~= int32(0)
        tracks.uvTracker(ddx) = peds_const.uvTracker;
    end

    cost(I_row, I_col) = peds_const.defNominalErr;
    costW(I_row, I_col) = single(peds_const.defNominalErr); 

    tracks = resetPedsTrackO(tracks, ddx);
    tracks = combinePedsTracksO(tracks, g_cfg_auto);
else
    cnt = 0;
    cost = peds_const.defNominalErr * ones(peds_const.track_width);
    costW = zeros(peds_const.track_width,'uint8');
end


end

