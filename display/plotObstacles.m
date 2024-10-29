function plotObstacles(ax, obs_XYs, radTar, stitle)
% 在直角坐标系内绘制障碍物
% obs_XYs   -   单帧检测到的障碍物
% radTar    -   雷达探测到的目标

global g_cfg_auto;
global g_tracks_auto;

cla(ax)
hold(ax, 'on')

% 单帧检测到的障碍物
x = zeros(1, obstacle_const.max_obs);
y = zeros(1, obstacle_const.max_obs);
cl = zeros(1, obstacle_const.max_obs, 'uint8');

% 被跟踪的目标
ex = zeros(1, obstacle_const.track_width);
ey = zeros(1, obstacle_const.track_width);
ec = zeros(1, obstacle_const.track_width, 'uint8');
sid = cell(1, obstacle_const.track_width);

% kalman滤波结果
kx = zeros(1, obstacle_const.track_width);
ky = zeros(1, obstacle_const.track_width);

vx = zeros(2, obstacle_const.track_width);
vy = zeros(2, obstacle_const.track_width);


cnt = 0;    % 单帧检测到的目标数量
ecnt = 0;   % 跟踪库中的有效目标
vcnt = 0;   % 跟踪库中的稳定跟踪目标
for j=1:obstacle_const.max_obs
    if isfield(obs_XYs, 'flag')
        if obs_XYs(j).flag
            cnt = cnt + 1;
            x(cnt) = obs_XYs(j).x;
            y(cnt) = obs_XYs(j).y;
            cl(cnt) = obs_XYs(j).cls+uint8(1);
        end
    end
end
df = 1;
for j=1:obstacle_const.track_width
    if g_tracks_auto.status(j).state >= track_status.initial
        ecnt = ecnt + 1;
%         sid{ecnt} = sprintf('%d(%03d, -%1d)(%.2f, %.2f)(%.2f, %.2f)', ...
%             j, g_tracks_auto.status(j).CNT, g_tracks_auto.status(j).LST, ...
%             g_tracks_auto.output(j).x, g_tracks_auto.output(j).y, ...
%             g_tracks_auto.output(j).rel_vx, g_tracks_auto.output(j).rel_vy);

        sid{ecnt} = sprintf('%d(%03d, -%1d)', ...
            j, g_tracks_auto.status(j).CNT, g_tracks_auto.status(j).LST);
        ex(ecnt) = g_tracks_auto.output(j).x;
        ey(ecnt) = g_tracks_auto.output(j).y;
        ec(ecnt) = g_tracks_auto.output(j).state;
        kx(ecnt) = g_tracks_auto.x_est(1,1,j);
        ky(ecnt) = g_tracks_auto.x_est(2,1,j);
        if g_tracks_auto.status(j).state >= track_status.stable
            vcnt = vcnt + 1;
            vx(:, vcnt) = [ex(ecnt), ex(ecnt) + g_tracks_auto.output(j).rel_vx * df];
            vy(:, vcnt) = [ey(ecnt), ey(ecnt) + g_tracks_auto.output(j).rel_vy * df];
        end
    end
end

c_sta = [ ...
    [0.7 0.7 0.7]; ...  % initial
    [0.8 0.8 0]; ...    % growing
    [0 1 0]; ...        % stable
    [1 0 0] ...         % unstable
    ];
c_cls = [...
    [0.7 0.7 0.7]; ...  % unknown
    [0 1 0]; ...        % car
    [0 0.8 0.8]; ...   	% bus
    [0 0 0.6]; ...     	% truck
    [1 0 0]; ...        % pedestrian
    [1 0 1] ...         % rider
    ];
x(cnt+1:end) = [];
y(cnt+1:end) = [];
cl(cnt+1:end) = [];
ex(ecnt+1:end) = [];
ey(ecnt+1:end) = [];
ec(ecnt+1:end) = [];
kx(ecnt+1:end) = [];
ky(ecnt+1:end) = [];
sid(ecnt+1:end) = [];
vx(:, vcnt+1:end) = [];
vy(:, vcnt+1:end) = [];


if cnt > 0
%     scatter(ax, y, x, [], c_cls(cl,:), 'filled')
    plot(ax, y, x, 'o')
end
if ecnt > 0
%     scatter(ax, ey, ex, 120, c_sta(ec,:), 's', 'linewidth', 2)
    plot(ax, ey, ex, 's')
    text(ax, ey, ex, sid, 'interpreter', 'latex')
%     scatter(ax, ky, kx, 'd')
    plot(ax, ky, kx, 'd')
end
if vcnt > 0
    line(vy, vx, 'color', 'r', 'linewidth', 1)
end

if ~isempty(radTar)
    N = numel(radTar);
    x = zeros(1,N);
    y = zeros(1,N);
    vx = zeros(1,N);
    vy = zeros(1,N);
    for i=1:N
        x(i) = radTar(i).x;
        y(i) = radTar(i).y;
        vx(i) = radTar(i).vx;
        vy(i) = radTar(i).vy;
    end
    scatter(ax, y, x, 'p')
end

set(ax, 'xdir', 'reverse', 'TickLabelInterpreter', 'latex')
title(ax, stitle, 'Interpreter', 'latex')
xlim(ax, g_cfg_auto.y_coverage)
ylim(ax, g_cfg_auto.x_coverage)
box(ax, 'on')
grid(ax, 'on')

end

