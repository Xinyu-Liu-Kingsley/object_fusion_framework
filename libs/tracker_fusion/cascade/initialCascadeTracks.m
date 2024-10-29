% Description: 初始化级联匹配的输入tracks
% Author: Zhou Yanzong
% log:
% 20230731 : created 
%****************************************************************%
function s  = initialCascadeTracks(n)
% 初始化目标列表

s.x         = zeros(n, 1, 'single');
s.y         = zeros(n, 1, 'single');
s.vx         = zeros(n, 1, 'single');
s.vy         = zeros(n, 1, 'single');
s.camID     = zeros(n,1,'uint16');
s.radID     = zeros(n,1,'uint16');
s.isUse     = zeros(n, 1, 'uint8');
s.state     = zeros(n, 1, 'uint8');
s.lostTimes = zeros(n, 1, 'uint8');
s.cnt       = zeros(n,1,'single');
s.det_src       = repmat(det_src.unknown,n,1);
end