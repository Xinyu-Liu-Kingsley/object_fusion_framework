function [flag_tracks] = get_flag_tracks(tracks)
% 表征轨迹状态的标志位
% 1- 新创建轨迹；
% 2- 正常更新轨迹；
% 3- 当前帧丢失；
% 4- 由上一帧丢失转为更新；
flag_tracks = zeros(obstacle_const.track_depth,1,'uint8');
if tracks.cursor == uint8(1)
    cursor_last = obstacle_const.track_depth;
else
    cursor_last = tracks.cursor-uint8(1);
end

for  i = 1: obstacle_const.track_width
    if tracks.status(i).flag
        if tracks.status(i).CNT== uint8(1)
            flag_tracks(i) = uint8(1);
        elseif tracks.lstflg(cursor_last, i)==0 && tracks.lstflg(tracks.cursor, i)==0
            flag_tracks(i) = uint8(2);
        elseif tracks.lstflg(tracks.cursor, i)==uint8(1)
            flag_tracks(i) = uint8(3);
        elseif tracks.lstflg(cursor_last, i)==uint8(1) && tracks.lstflg(tracks.cursor, i)==0
            flag_tracks(i) = uint8(4);
        end
    end
end

end

