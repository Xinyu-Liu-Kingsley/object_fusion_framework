function ego_status = update_ego(ego_status,... % 车辆运行状态（一般为全局变量）
    speed, vx, vy, ax, ay, yawrate, SW_angle, SW_alpha, BPP, APP, wiper, indicator)    %#codegen
  % Description: 更新车辆运行状态
% Author :  Shane Liu
% Modify Note: 
% ************************************************************ % 
coder.inline('always')

assert(isscalar(speed) && isscalar(vx) && isscalar(vy) && isscalar(ax)...
    && isscalar(ay) && isscalar(yawrate) && isscalar(SW_angle) ...
    && isscalar(SW_alpha) && isscalar(BPP) && isscalar(APP) ...
    && isscalar(wiper) && isscalar(indicator))
assert(isa(wiper, 'uint8') && isa(indicator, 'uint8'))

ego_status.speed = check_range(speed, ego_const.rng_speed);
ego_status.vx = check_range(vx, ego_const.rng_vx);
ego_status.vy = check_range(vy, ego_const.rng_vy);
ego_status.ax = check_range(ax, ego_const.rng_ax);
ego_status.ay = check_range(ay, ego_const.rng_ay);
ego_status.SW_angle = check_range(SW_angle, ego_const.rng_SW_angle);
ego_status.SW_alpha = check_range(SW_alpha, ego_const.rng_SW_alpha);

% update egoCar yawrate history
ego_status.yawrateHis(ego_status.cursor) = check_range(yawrate, ego_const.rng_yawrate); % deg
ego_status.cursor = ego_status.cursor + uint8(1);
if ego_status.cursor > ego_const.len_yaw_history
    ego_status.cursor = uint8(1);
end
% 判断有无yawrate
if abs(max(ego_status.yawrateHis)-min(ego_status.yawrateHis))<1e-6
    ego_status.yawrate = 0;
else
    ego_status.yawrate = check_range(yawrate, ego_const.rng_yawrate);
end

% update egoCar speed history (10 frames)
vx_history = ego_status.vx_history;
ego_status.vx_history(end) = vx;
ego_status.vx_history(1:end-1) = vx_history(2:end);

ego_status.BPP = check_range(BPP, ego_const.rng_BPP);  % Brake Pedal Position
ego_status.BPS = uint8(ego_status.BPP >= 5);    % 踏板位置零漂
ego_status.APP = check_range(APP, ego_const.rng_APP);
ego_status.wiper = wiper;
ego_status.indicator = indicator;

if speed > 0  
    ego_status.curvature = deg2rad(mean(ego_status.yawrateHis)) / (speed / 3.6);
else
    ego_status.curvature = 0;
end

end

