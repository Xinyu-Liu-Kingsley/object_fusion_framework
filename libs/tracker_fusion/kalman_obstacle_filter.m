
function [tracks, est] = kalman_obstacle_filter(tracks, cfg, x, y, vx, vy, ax, ay, jdx, init)    %#codegen
%#codegen

% global g_ego_status;
narginchk(9,10)
if nargin < 10, init = 0; end

maxX_R = 40; % 近端使用Rx的范围
est = zeros(1, 6);

% Initialize state transition matrix
% global g_cfg_auto;
dt = cfg.deltaT;
A = [ 1 0 dt 0  dt^2/2 0;...          % [x ]            
 	  0 1 0  dt 0      dt^2/2;...     % [y ]
  	  0 0 1  0  dt     0;...          % [Vx]
   	  0 0 0  1  0      dt;...         % [Vy]
      0 0 0  0  1      0;...
      0 0 0  0  0      1];        
H = [ 1 0 0 0 0 0; 0 1 0 0 0 0; 0 0 1 0 0 0; 0 0 0 1 0 0 ];    % Initialize measurement matrix
% varaX = cfg.varaX;
% varaY = cfg.varaY;
Q = eye(6)*0.04;
% varY = 2;
% Q=[ varaX*dt^6/36,	 0,                varaX*dt^5/12,	 0               varaX*dt^4/6     0;...
%     0,               varaY*dt^6/36,    0,                varaY*dt^5/12   0                varaY*dt^4/6;...
%     varaX*dt^5/12,   0,                varaX*dt^4/4,     0               varaX*dt^3/2     0;...
%     0,               varaY*dt^5/12,    0,                varaY*dt^4/4    0                varaY*dt^3/2;...
%     varaX*dt^4/6,    0,                varaX*dt^3/2,     0               varaX*dt^2       0;...
%     0,               varaY*dt^4/6,     0,                varaY*dt^3/2    0                varaY*dt^2];

%R = cfg.R* eye(2);
r = tracks.x_est(1,1,jdx)*cfg.r_ratio;
if  tracks.status(jdx).state < track_status.stable || tracks.x_est(1,1,jdx)<maxX_R
    r = cfg.Rx;
end
R = [ r 0 0 0; 0 cfg.Ry 0 0; 0 0 cfg.Rvx 0; 0 0 0 cfg.Rvy];

% 初始化状态创建
if init > 0
    % TTC 2.7s，以4为时间 
%     x = g_ego_status.x;
%     y = g_ego_status.y;
%     vx = -1*g_ego_status.vx;
%     vy = g_ego_status.vy;
%     vy = -1*min([g_ego_status.vy, 5, x/4]); %-g_ego_status.vx;
%     ax = g_ego_status.ax;
%     ay = g_ego_status.ay;
    tracks.x_est(:,:,jdx) = [x, y, vx, vy, ax, ay]';
    tracks.p_est(:,:,jdx) = Q;
    return;
end 

% Predicted state and covariance
x_prd = A * tracks.x_est(:,:,jdx);
p_prd = A * tracks.p_est(:,:,jdx) * A' + Q;

% Estimation
S = H * p_prd' * H' + R;
B = H * p_prd';
klm_gain = (S \ B)';

% Estimated state and covariance
tracks.x_est(:,:,jdx) = x_prd + klm_gain * ([x; y; vx; vy] - H * x_prd);
tracks.p_est(:,:,jdx) = p_prd - klm_gain * H * p_prd;

% Compute the estimated measurements
% est = H * x_est(:,:,jdx);
est = (tracks.x_est(:,:,jdx))';

end                % of the function
