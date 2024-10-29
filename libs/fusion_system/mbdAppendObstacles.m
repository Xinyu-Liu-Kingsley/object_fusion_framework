% Description: 航迹跟踪器
% Author :  Shane Liu
% Modify Note: 
% ************************************************************ % 
function out_auto = mbdAppendObstacles(obs_XYs,frameData)    %#codegen
% 跟踪库 ，生命周期管理
% 区分行人 车辆；
global g_tracks_auto g_cfg_auto;

coder.cstructname(obs_XYs,'Obstacles')

assert(numel(obs_XYs) == obstacle_const.max_obs_fuse)
[m, n] = size(g_tracks_auto.matrix);
assert(m == obstacle_const.track_depth)
assert(n == obstacle_const.track_width)
assert(g_tracks_auto.cursor >= 1 && g_tracks_auto.cursor <= obstacle_const.track_depth)

obs_auto = obs_XYs;

% 数据点处理标识（匹配成功，或创建新的轨迹）。0 - 表示尚未处理该数据点；1 - 表示已处理该数据点。
flags_auto = uint8(zeros(1, obstacle_const.max_obs_fuse));
% isAuto = true;

% 预测轨迹
g_tracks_auto = predictTracksO(g_tracks_auto, g_cfg_auto);
% 遍历障碍物轨迹库
[g_tracks_auto, flags_auto, ~] = assignTracksO(g_tracks_auto, obs_auto, flags_auto);
% 合并疑似同一目标的轨迹(之前未关联，后来关联上的目标)
g_tracks_auto = combineTracksO(g_tracks_auto, g_cfg_auto);
% 维护未匹配成功的轨迹
g_tracks_auto = repaireTracksO(g_tracks_auto);
% 删除丢失的轨迹
g_tracks_auto = deleteTracksO(g_tracks_auto, g_cfg_auto);
% 创建新的轨迹
g_tracks_auto = createTracksO(g_tracks_auto, g_cfg_auto, obs_auto, flags_auto);
% 基于EKF更新x vx
% g_tracks_auto = updateTracksEKF(g_tracks_auto,g_cfg_auto,isAuto);
% 更新轨迹库
[g_tracks_auto, isFill] = updateTracksO(g_tracks_auto, g_cfg_auto);
% 按照协议要求输出
out_auto = updateOutVeh(g_tracks_auto,frameData,isFill);
 
coder.cstructname(out_auto, 'ObstacleEOutV')

end


