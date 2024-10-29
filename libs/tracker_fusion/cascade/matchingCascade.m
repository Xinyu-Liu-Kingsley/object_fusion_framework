% Description: 级联匹配算法
% Author: Zhou Yanzong
% log:
% 20230731 : created
%****************************************************************%
function [M] = matchingCascade(tracks_cas, IDmatrix, maxAge, maxDist, feature, cosThreshold)
% 级联匹配
% costMatrix自定义maxDist*ones，MXN，赋值输出成本矩阵

% 初始化
nMeas       = uint8(size(feature, 1));
nTrs        = uint8(size(feature, 2));
% B2          = zeros(nMeas, nTrs, 'uint8');
% B3          = zeros(nMeas, nTrs, 'uint8');
M           = zeros(nMeas, nTrs, "uint8");
costMatrix  = single(maxDist)*ones(nMeas, nTrs, 'single');
% unmatchD    = zeros(nMeas, 1, 'uint8');
% unmatchT    = zeros(nTrs, 1, 'uint8');

% % uv距离关联指示B2
% B2(uvDist < maxDist) = uint8(1);
% feature 关联指示
% B3(feature > cosThreshold) = uint8(1);
% % 联合关联指示
% B = B3;
% featureDist(B < 1) = single(maxDist);
feature(feature > cosThreshold) = single(maxDist);
% 距离上次匹配距离越近的，优先匹配，最近邻匹配
% 稳定航迹ID关联
for n = uint8(1:maxAge)
    for i = 1:nTrs
        if tracks_cas.isUse(i) >= 1 %&& tracks_cas.state(i) >= stableSta
            if tracks_cas.lostTimes(i) + 1 == n
                if any(IDmatrix(:,i)>0)   % 如果ID assign能够匹配上，不参与后续的匹配
                    [id, ~]= find(IDmatrix(:,i)>0);
                    id = single(id);
                    value = single(1);
                    feature(id(1), (1:nTrs)) = single(maxDist);  % 匹配的行和列置最大值
                    feature((1:nMeas),i) = single(maxDist);
                    costMatrix(id(1),i) = single(value);
                end
            end
        end
    end
end

% 最近邻匹配
% for n = uint8(1:maxAge)
%     for i = 1:nTrs
%         if tracks_cas.isUse(i) >= 1 && tracks_cas.state(i) >= stableSta
%             if tracks_cas.lostTimes(i) + 1 == n
%                 [value, id] = min(feature(:,i));
%                 value = single(value);
%                 id = single(id);
%                 if value < maxDist
%                     feature(id(1), (1:nTrs) ~= i) = single(maxDist);
%                     costMatrix(id(1),i) = single(value);
%                 end
%             end
%         end
%     end
% end
% 输出匹配结果
M(costMatrix < maxDist) = uint8(1);

% 未匹配的detection, unmatchD(j) = 1;
% for j = 1:nMeas
%     if sum(M(j,:)) > 0
%         unmatchD(j) = uint8(0);
%     else
%         unmatchD(j) = uint8(1);
%     end
% end
% % 未匹配的tracks， unmatchT(i) = 1;
% for i = 1:nTrs
%     if sum(M(:,i)) > 0
%         unmatchT(i) = uint8(0);
%     else
%         unmatchT(i) = uint8(1);
%     end
% end

end