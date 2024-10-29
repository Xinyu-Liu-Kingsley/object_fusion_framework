function tracks =  predictTracksO(tracks, cfg)    %#codegen
% 预测障碍物下一时刻状态

for i=1:obstacle_const.track_width
    if tracks.status(i).used == uint8(1)
        
        tracks.estimated(i).x = cfg.deltaT * ...
            tracks.estimated(i).velo_x + tracks.estimated(i).x; % 预测x位置
        tracks.estimated(i).y = cfg.deltaT * ...
            tracks.estimated(i).velo_y + tracks.estimated(i).y; % 预测y位置
        tracks.estimated(i).velo_y = cfg.deltaT ...
        *tracks.estimated(i).ay+tracks.estimated(i).velo_y; % 预测纵向速度
    else
        tracks.estimated(i) = obstacle_const.FusedObject;  % 初始估计值
    end
end

end
