
function eobstacles= mbdUpdateFusion(frameData)
%  MBDPROCESSDATA 此处显示有关此函数的摘要
%  对雷达和摄像头的数据进行融合，返回融合的ObjecetList
coder.cstructname(frameData, 'FrameData')
assert(numel(frameData.CameraFrame.CameraObjectList) == obstacle_const.max_obs_camera)
assert(numel(frameData.RadarFrame.RadarObjectList) == obstacle_const.max_obs_radar)
% 雷达坐标系转换
%  frameData = mbdProcessData(frameData);
% Radar & Camera融合模块
obstacles = mbdFuseObstacels(frameData);
% update Fusion结果
[eobstacles] = mbdAppendObstacles(obstacles,frameData);
end
