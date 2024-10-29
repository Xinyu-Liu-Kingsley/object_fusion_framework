% Description: 级联匹配模块
% Author:Zhou Yanzong
% 前后帧只做ID关联，避免同传感器的误关联
% log:
% 20230728 : created
%****************************************************************%
function matchM = cascadeFun(tracks_cas, dets_cas, numT, numM, param)
% 提取距离和速度特征 x,y vx,vy
feature = calcObjsFeatures(tracks_cas, dets_cas, numM, numT, param.featureWeight, param.lateralDistThres);
% 计算ID matrix
IDmatrix = calcIDMatrix(tracks_cas, numT, dets_cas, numM);
% 级联匹配
[matchM] = matchingCascade(tracks_cas,IDmatrix, param.maxLostAge, param.maxDist, feature, param.cosThreshold);
% calc xy
% costMatrix = calcEuclideanMatrix(tracks_cas, dets_cas, unmatchD, unmatchT,  param);
% hungarian
% [matching, ~] = munkres(costMatrix);
% output match matrix
% matchM = M;
% matchM = mergeMatching(M, matching, costMatrix);

end

% function matchM = mergeMatching(M, matching, costMatrix)
% % matching:匈牙利出来的二进制矩阵，需要构建B指示矩阵滤除干扰
% % M:级联匹配结果，二进制矩阵，已经滤除干扰，可以直接使用
% 
% B = zeros(size(costMatrix), 'uint8');
% B(costMatrix < 10000) = uint8(1);
% matching = B .* uint8(matching);
% % 最终匹配结果
% matchM = M | matching;
% 
% end