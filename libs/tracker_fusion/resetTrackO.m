function tracks = resetTrackO(tracks, jdx)    %#codegen

% global g_tracks_auto;
assert(jdx >= 1 && jdx <= obstacle_const.track_width)

tracks.status(jdx).CNT = int32(0);
tracks.status(jdx).LST = int32(0);
tracks.status(jdx).CNTCam = int32(0);
tracks.status(jdx).CNTRad = int32(0);
tracks.status(jdx).CNTFused = int32(0);
tracks.status(jdx).LSTCam = int32(0);
tracks.status(jdx).LSTRad = int32(0);
tracks.status(jdx).used = uint8(0);  
tracks.status(jdx).state = track_status.untracking;
% tracks.status(jdx).cls = obstacle_cls.unknown;
tracks.status(jdx).flag = uint8(0);


tracks.status(jdx).y_history = zeros(10,1);
tracks.status(jdx).CamID_history = zeros(10,1,'uint8');
tracks.status(jdx).RadID_history = zeros(10,1,'uint16');
tracks.status(jdx).rel_status = uint8([0 0]);

tracks.estimated(jdx) = obstacle_const.FusedObject;
tracks.output(jdx) = obstacle_const.FusedObject;

tracks.matrix(:, jdx) = repmat(obstacle_const.FusedObject, obstacle_const.track_depth, 1);
tracks.lstflg(:, jdx) = zeros(obstacle_const.track_depth, 1, 'uint8');


end

