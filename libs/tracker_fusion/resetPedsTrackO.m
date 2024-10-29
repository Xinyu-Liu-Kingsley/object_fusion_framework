function tracks = resetPedsTrackO(tracks, jdx)    %#codegen

% global g_tracks_auto;
assert(jdx >= 1 && jdx <= peds_const.track_width)

tracks.status(jdx).CNT = int32(0);
tracks.status(jdx).LST = int32(0);
tracks.status(jdx).used = uint8(0);  
tracks.status(jdx).state = track_status.untracking;
tracks.status(jdx).cls = obstacle_cls.unknown;
tracks.status(jdx).flag = uint8(0);

tracks.status(jdx).clsCNT = int32(0);
tracks.status(jdx).obsW =  single(0);
tracks.status(jdx).cls1 = obstacle_cls.unknown;
tracks.status(jdx).cls1CNT = int32(0);

tracks.estimated(jdx) = peds_const.EO;
tracks.output(jdx) = peds_const.EO;

tracks.matrix(:, jdx) = repmat(peds_const.XY, peds_const.track_depth, 1);
tracks.lstflg(:, jdx) = zeros(peds_const.track_depth, 1, 'uint8');


end

