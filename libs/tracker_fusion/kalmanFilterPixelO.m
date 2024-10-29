% Description: kalman Filter for boundingboxes
% Author : hanhua                    
% log:
% ...
%****************************************************************% 
function [corrX,corrP,predZ] = kalmanFilterPixelO(uv,X,P,stateFlag)   % codegen      % K-1时刻
% 图像坐标系下目标框跟踪
% 跟踪状态：xywh(中心坐标，宽，高) state：[x,y,a,h,vx,vy,va,vh]' ;
% measurement :[x,y,w,h]
% 修正状态参数为：(中心坐标，宽，高)
% [x,y,w,h,vx,vy,vw,vh]


% state transition
dt = single(1/15);

F = single([1 0 0 0 dt 0 0 0; ...
            0 1 0 0 0 dt 0 0; ...
            0 0 1 0 0 0 dt 0; ...
            0 0 0 1 0 0 0 dt; ...
            0 0 0 0 1 0 0 0; ...
            0 0 0 0 0 1 0 0; ...
            0 0 0 0 0 0 1 0; ...
            0 0 0 0 0 0 0 1 ]);
% measurement
H = single([1 0 0 0 0 0 0 0; ...
            0 1 0 0 0 0 0 0; ...
            0 0 1 0 0 0 0 0; ...
            0 0 0 1 0 0 0 0]);   % z = H*x
% measurement noise
R = single(diag([1 1 1 1]));  % [x,y,a,h]

% process noise
varX = single(5);
varY = single(10);
varA = single(30); % 宽高比
varH = single(30);

Q = single([varX*dt^4/4  0  0  0  varX*dt^3/2 0 0 0 ;...
            0 varY*dt^4/4 0 0  0 varY*dt^3/2 0 0;...
            0 0 varA*dt^4/4 0 0 0 varA*dt^3/2 0; ...
            0 0 0 varH*dt^4/4 0 0 0 varH*dt^3/2; ...
            varX*dt^3/2 0 0 0 varX*dt^2 0 0 0; ...
            0 varY*dt^3/2 0 0 0 varY*dt^2 0 0 ;...
            0 0 varA*dt^3/2 0 0 0 varA*dt^2 0; ...
            0 0 0 varH*dt^3/2, 0 0 0 varH*dt^2]);

% 测量值  uv(u,v,w,h)
% Z = single([(uv(1)*2+uv(3))/2; (uv(2)*2+uv(4))/2; ...
%     uv(3)/(uv(4)+1e-7); uv(4)]);
Z = single([(uv(1)*2+uv(3))/2; (uv(2)*2+uv(4))/2; ...
    uv(3); uv(4)]);

if  stateFlag == obstacle_const.uvTrackerStatus.init
    corrX = single([Z;0;0;0;0]);
    corrP = single(diag([diag(R);1;1;1;1]));
    % 预测下一时刻观测值
    predZ = H*(F*single(corrX));
    return;
end

% predicte X and P; estimation
predX = F*single(X);
predP = F*single(P)*F'+ Q;

% 当前未检测到，预测值做为当前更新状态值输出
if stateFlag == obstacle_const.uvTrackerStatus.pred
    corrX = predX;
    corrP = predP;
else

    % correction X and P
    K = (predP*H')/(H*predP*H'+R+1e-7);
    corrX = predX + K*(Z-H*predX);
    corrP = predP- K*H*predP;
end


% 预测下一时刻观测值
predZ = H*(F*single(corrX));


% 限制条件 （h>0）(w>0)
if corrX(4)<0
    corrX(4) = single(0);
end
if predZ(4)<0
    predZ(4) = single(0);
end
if corrX(3)<0
    corrX(3) = single(0);
end
if predZ(3)<0
    predZ(3) = single(0);
end

end