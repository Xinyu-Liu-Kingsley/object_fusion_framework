function [lanes_uv, lanes_xy, lanes_me, eLanes, obstacles_uv, obstacles, eobstacles, pitches] = extraBathcing(path, name_without_ext, frmNum, cam, uifig)
%EXTRABATHCING 此处显示有关此函数的摘要
%   此处显示详细说明

if coder.target('MATLAB')
    if nargin < 1
        uifig = [];
        frmNum = int32(100);
        if ismac
            path = '/Users/danjiabi/RawData/CalmCar/';
        else
            path = 'C:\Users\danjiabi\RawData\CalmCar\';
        end
        name_without_ext = '2018-09-06-11-41-52_fcm';
        cam = mbdCreateCamera((1444.70896), (1440.42575), (654.12586), (358.51293),...
            int32([720, 1280]), (1.72), (0), (0), (-6.3), (0), (0));
    end
end

if ~isempty(uifig)
    d = uiprogressdlg(uifig, 'Title', '检测信息处理中...', 'Cancelable', 'on');
end

[lanes_nn, lanes_uv, obstacles_uv, L, error] = auxLoadData(fullfile(path, name_without_ext), int32(1), int32(frmNum), int32(1));
assert(size(lanes_uv,1) == size(obstacles_uv,1) && size(lanes_uv, 1) == L)

lanes_xy = repmat(lane_const.XY, L, lane_const.max_lns);
lanes_me = repmat(lane_const.ME, L, lane_const.max_lns);
eLanes = repmat(lane_const.EO, L, lane_const.track_width);
pitches = zeros(L, 1);

obstacles = repmat(obstacle_const.XY, L, obstacle_const.max_obs);
eobstacles = repmat(obstacle_const.EO, L, obstacle_const.track_width);

if error == uint8(0)
    for i=1:L
        if ~isempty(uifig)
            if d.CancelRequested, break; end
            d.Value = double(i)/double(L);
            d.Message = sprintf('%d/%d...', i, L);
        end
        [lanes_me(i, :), lanes_xy(i, :)] = mbdProcessLanes(lanes_uv(i, :));
        eLanes(i,:) = mbdAppendLanes(lanes_me(i, :));
        
        pitches(i) = cam.last_pitch;
        
        obstacles(i,:) = mbdProcessObstacles(obstacles_uv(i,:));
        eobstacles(i,:) = mbdAppendObstacles(obstacles(i, :));
    end
else
    error('装载本地数据文件出错！')
end

if ~isempty(uifig)
    close(d);
end

if ~isempty(uifig)
    d = uiprogressdlg(uifig, 'Title', '保存至本地...', 'Indeterminate', 'on');
end

camera = cam;
save(fullfile(path, [name_without_ext, '.4app']), 'camera', 'obstacles_uv', 'obstacles', 'eobstacles', 'lanes_uv', 'lanes_xy', 'lanes_me', 'pitches', 'eLanes') 

if ~isempty(uifig)
    close(d);
end

end

