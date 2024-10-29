% Description: 处理已关联的目标，并更新跟踪库
% Author: Zhou Yanzong
% log:
% 20230731 : created
%****************************************************************%
function [measAssociateFlag, trsAssociateFlag, tracks] = flagAssociatedTracks(tracks, rawMeas, MM, measAssociateFlag)
% 处理已关联的目标，并更新跟踪库
% 输入tracks: tracks
% 输入rawMeas:radar +camera objects
% measAssociateFlag:记录detections关联与否，1-关联，0-未关联
% trsAssociateFlag:记录tracks关联与否，1-关联，0-未关联

%初始化
numM = size(MM, 1);
numT = size(MM, 2);
trsAssociateFlag = zeros(numT, 1, 'uint8');

% update flag
for i = 1:numM
    if (any(MM(i,:) > 0))
        trsId = find(MM(i,:) > 0);
    else
        continue;
    end
    [tracks, rawMeas(i)] = assigned(tracks, trsId(1), rawMeas(i));
    measAssociateFlag(i) = uint8(1);
    trsAssociateFlag(trsId(1)) = uint8(1);
end

end