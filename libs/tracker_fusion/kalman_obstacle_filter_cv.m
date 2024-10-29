function [tracks, est] = kalman_obstacle_filter(tracks, cfg, x, y, jdx, init)    %#codegen
%#codegen

global g_ego_status;
narginchk(5,6)
if nargin < 6, init = 0; end

maxX_R = 40; % 近端使用Rx的范围
est = zeros(1, 4);

% Initialize state transition matrix
% global g_cfg_auto;
dt = cfg.deltaT;
A = [ 1 0 dt 0 ;...     % [x ]            
 	  0 1 0  dt;...     % [y ]
  	  0 0 1  0 ;...     % [Vx]
   	  0 0 0  1];        % [Vy]
H = [ 1 0 0 0; 0 1 0 0 ];    % Initialize measurement matrix
varX = cfg.varX;
varY = cfg.varY;
% varY = 2;
Q=[ varX*dt^4/4,	0,              varX*dt^3/2,	0;...
    0,              varY*dt^4/4,    0,              varY*dt^3/2;...
    varX*dt^3/2,    0,              varX*dt^2,      0;...
    0,              varY*dt^3/2,    0,              varY*dt^2];

%R = cfg.R* eye(2);
r = tracks.x_est(1,1,jdx)*cfg.r_ratio;
if  tracks.status(jdx).state < track_status.stable || tracks.x_est(1,1,jdx)<maxX_R
    r = cfg.Rx;
end
R = [ r 0; 0  cfg.Ry];

% 初始化状态创建
if init > 0
    % TTC 2.7s，以4为时间 
    vy = -1*min([g_ego_status.vy, 5, x/4]); %-g_ego_status.vx;
    tracks.x_est(:,:,jdx) = [x, y, 0, vy]';
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
tracks.x_est(:,:,jdx) = x_prd + klm_gain * ([x; y] - H * x_prd);
tracks.p_est(:,:,jdx) = p_prd - klm_gain * H * p_prd;

% Compute the estimated measurements
% est = H * x_est(:,:,jdx);
est = (tracks.x_est(:,:,jdx))';

end                % of the function
