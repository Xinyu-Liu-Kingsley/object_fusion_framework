function tracks =  predictPedsTracksO(tracks, cfg)    %#codegen
% 预测障碍物下一时刻状态

for i=1:peds_const.track_width
    if tracks.status(i).used == uint8(1)
        
        tracks.estimated(i).x = cfg.deltaT * ...
            tracks.estimated(i).rel_vx + tracks.estimated(i).x;
        tracks.estimated(i).y = cfg.deltaT * ...
            tracks.estimated(i).rel_vy + tracks.estimated(i).y;
    else
        tracks.estimated(i) = peds_const.EO;
    end
end

end
