function plotLanes(ax, lanes, lanes_xy, stitle)
% 直角坐标系内绘制车道线

global g_cfg_lane;

leg = cell(lane_const.max_lns*2,1);
cnt = 0;
cnt1 = 0;
cnt2 = 0;

cla(ax)
hold(ax, 'on')

if ~isempty(lanes_xy)
    if isfield(lanes_xy, 'x') && isfield(lanes_xy, 'y')
        for j=1:lane_const.max_lns
            len = lanes_xy(j).len;
            if len > 0
%                 scatter(ax, lanes_xy(j).y(1:len), lanes_xy(j).x(1:len))%, 'filled'
                plot(ax, lanes_xy(j).y(1:len), lanes_xy(j).x(1:len), ':')%, 'filled'
                cnt = cnt+1;
                cnt1 = cnt1+1;
                leg{cnt} = sprintf('Cluster.%d', cnt1);
            end
        end
    end
end

wdLines = [0.5, 2, 2, 1, 1];
crLines = {[0.5 0.5 0.5], [1 0 0], [0 1 1], [1 0 0], [0 1 1]};
stLines = {':', '-.', '-.', '--', '--'};
if ~isempty(lanes)
    if isfield(lanes, 'C') && isfield(lanes, 'range') && isfield(lanes, 'flag')
        for j=1:lane_const.max_lns
            if lanes(j).flag
                y = linspace(0, lanes(j).range(2), lane_const.max_pts);
%                 y = linspace(lanes(j).range(1), lanes(j).range(2), lane_const.max_pts);
%                 y = linspace(0, 80, lane_const.max_pts);
                x = polyval(lanes(j).C(4:-1:1), y);
                plot(x, y, 'linestyle', stLines{lanes(j).index+1}, 'linewidth', wdLines(lanes(j).index+1), 'color', crLines{lanes(j).index+1})
                cnt = cnt+1;
                cnt2 = cnt2+1;
                leg{cnt} = sprintf('Line.%d', cnt2);
            end
        end
    end
    if isfield(lanes, 'C') && isfield(lanes, 'range') && isfield(lanes, 'state')
        for j=1:lane_const.max_lns
            if lanes(j).state > track_status.initial
                y = linspace(0, lanes(j).range(2), lane_const.max_pts);
%                 y = linspace(lanes(j).range(1), lanes(j).range(2), lane_const.max_pts);
%                 y = linspace(0, 80, lane_const.max_pts);
                x = polyval(lanes(j).C(4:-1:1), y);
                plot(x, y, 'linestyle', stLines{lanes(j).index+1}, 'linewidth', wdLines(lanes(j).index+1), 'color', crLines{lanes(j).index+1})
                cnt = cnt+1;
                cnt2 = cnt2+1;
                leg{cnt} = sprintf('Line.%d', cnt2);
            end
        end
    end
end

leg(cnt+1:end) = [];
legend(leg, 'NumColumns', 3, 'Interpreter', 'latex')
legend('boxoff')

set(ax, 'xdir', 'reverse', 'TickLabelInterpreter', 'latex')
title(ax, stitle, 'Interpreter', 'latex')
xlabel(ax, '$Y$ / m', 'Interpreter', 'latex')
ylabel(ax, '$X$ / m', 'Interpreter', 'latex')
xlim(ax, g_cfg_lane.y_coverage*3)
ylim(ax, g_cfg_lane.x_coverage)
box(ax, 'on')
grid(ax, 'on')
% axis(ax, 'equal')

end

