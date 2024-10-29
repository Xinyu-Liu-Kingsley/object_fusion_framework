% Description: 计算XY矩阵
% Author: Zhou Yanzong
% log:
% 20230728 : created 
%****************************************************************%
function costMatrix = calcEuclideanMatrix(tracks_cas, dets_cas, unmatchD, unmatchT, param)
% 计算XY
% costMatrix:计算x，y矩阵，关联门限放宽，对于新建航迹和丢失航迹
% unmatchD:级联匹配的输出，未匹配的detection flag,0-已经匹配，1-未匹配,Mx1
% unmatchT:级联匹配的输出，未匹配的tracks flag，0-已经匹配，1-未匹配，Nx1
numM = uint8(numel(unmatchD));
numT = uint8(numel(unmatchT));
costMatrix  = ones(numM, numT, 'single')*10000;

for i = 1:numM
    if unmatchD(i) == 0 || dets_cas.isUse(i) < 1
        continue;
    end
    for j = 1:numT
        if unmatchT(j) == 0
            continue;
        end
        if tracks_cas.isUse(j) >= 1
            [distance,disThres] = calcEuclideanDis(tracks_cas, j, dets_cas, i,param);
            if distance < disThres
                costMatrix(i,j) = distance;
            end
        end
    end
end

end

function [distance,disThres] = calcEuclideanDis(tracks_cas, iTrs, dets_cas, jMeas,param)
   % 按照距离段进行阈值计算
   xMeas = dets_cas.x(jMeas);
   yMeas = dets_cas.y(jMeas);
   xTrs =  tracks_cas.x(iTrs);
   yTrs =  tracks_cas.y(iTrs);
%    s_x = interp1(param.x, param.s_x, xTrs, 'linear', 0.2);
%    s_y = interp1(param.x, param.s_y, yTrs, 'linear', 0.2);
   distance= abs(xTrs - xMeas)  + abs(yTrs - yMeas);
   disThres = 10;
end