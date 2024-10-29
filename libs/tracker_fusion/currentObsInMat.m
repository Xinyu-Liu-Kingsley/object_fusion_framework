function obs_XY = currentObsInMat(tracks, idx, isAuto)    %#codegen
% 当前障碍物目标
%   输入：idx    	-   障碍物跟踪库ID，[1 ~ obstacle_const.track_width]
%   输出：obs_XY 	-   对应障碍物

assert(idx >= 1 && idx <= obstacle_const.track_width)
% if nargin <= 2
%     isAuto = true;
% end
% if isAuto == false
%     obs_XY = peds_const.XY;
% else
%     obs_XY = obstacle_const.XY;
% end
obs_XY = obstacle_const.FusedObject;
track_depth = obstacle_const.track_depth;

if tracks.status(idx).used
    if tracks.status(idx).flag
        obs_XY = tracks.matrix(tracks.cursor, idx);
    else
        obs_XY = tracks.matrix(prev_cursor(...
            tracks.cursor, track_depth), idx);
    end
    if tracks.status(idx).state >= track_status.stable
        obs_XY.x = tracks.estimated(idx).x;
        obs_XY.y = tracks.estimated(idx).y;
    end
end

end

