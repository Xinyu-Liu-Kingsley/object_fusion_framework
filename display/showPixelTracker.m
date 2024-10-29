function showPixelTracker(ax,frm)
%UNTITLED5 此处提供此函数的摘要
%   显示 uvTracker
global g_tracks_auto
hold(ax, 'on');

for i = 1:obstacle_const.track_width
    if g_tracks_auto.uvTracker(i).used
        X =  g_tracks_auto.uvTracker(i).X(1:4);

        %     assert(X(3)>0);
        %     assert(X(4)>0);
        if X(4)>0
            h = X(4);
            %w = X(3)*h;
            w = X(3);
            u = X(1)-w/2;
            v = X(2)-h/2;
            rectangle(ax,'Position', [u, v, w, h], 'EdgeColor', 'm','LineStyle','--');
        end
    end

end


end