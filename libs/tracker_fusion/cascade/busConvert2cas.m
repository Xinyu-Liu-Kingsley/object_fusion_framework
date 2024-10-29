% Description: 转为级联匹配的模块输入定义的结构体
% Author: Zhou Yanozong
% log:
% 20230731 : created
%****************************************************************%
function [tracks_cas, dets_cas] = busConvert2cas(tracks, obs, numT, numM)
% 转为级联匹配的模块输入

% 初始化
tracks_cas  = initialCascadeTracks(numT);
dets_cas    = initialCascadeTargets(numM);

for i = 1:numT
    tracks_cas.x(i)         = tracks.estimated(i).x;
    tracks_cas.y(i)         = tracks.estimated(i).y;
    tracks_cas.vx(i)        = tracks.estimated(i).velo_x;
    tracks_cas.vy(i)        = tracks.estimated(i).velo_y;
    tracks_cas.camID(i)        = tracks.output(i).fusedCamID;
    tracks_cas.radID(i)        = tracks.output(i).fusedRadID;
    tracks_cas.isUse(i)     = tracks.status(i).used;
    tracks_cas.state(i)     = tracks.status(i).state;
    tracks_cas.lostTimes(i) = tracks.status(i).LST;
    tracks_cas.cnt(i)       = tracks.status(i).CNT;
    tracks_cas.det_src(i)       = tracks.output(i).det_src;
end

for j = 1:numM
    dets_cas.x(j)       = obs(j).x;
    dets_cas.y(j)       = obs(j).y;
    dets_cas.vx(j)      = obs(j).velo_x;
    dets_cas.vy(j)      = obs(j).velo_y;
    dets_cas.camID(j)      = obs(j).fusedCamID;
    dets_cas.radID(j)      = obs(j).fusedRadID;
    dets_cas.isUse(j)   = obs(j).flag;
    dets_cas.det_src(j)   = obs(j).det_src;
end

end