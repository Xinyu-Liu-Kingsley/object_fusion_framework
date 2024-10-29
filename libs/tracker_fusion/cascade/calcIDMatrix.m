% Description: 计算Track 和detection ID匹配状况
% Author: Zhou Yanzong
% log:
% 20230818 : created
%****************************************************************%
function [IDMatrix] = calcIDMatrix(tracks_cas, numT, dets_cas, numM)
% 计算Track 和detection ID匹配状况
% IDMatrix:自定义的距离矩阵，将计算结果输出
% 输入tracks:融合目标跟踪库数据
% 输入obs_auto:当前帧标原始数据

IDMatrix = zeros(numM, numT);

for j = 1:numT
    if tracks_cas.isUse(j) <= 0
        continue;
    end
    trs_ID = [uint16(tracks_cas.camID(j)), uint16(tracks_cas.radID(j))];
    trs_det_src = tracks_cas.det_src(j);
    for i = 1:numM
        if dets_cas.isUse(i) <= 0
            continue;
        end
        meas_ID =  [uint16(dets_cas.camID(i)), uint16(dets_cas.radID(i))];
        meas_det_src = dets_cas.det_src(i);
%         IDmatch = trs_ID - meas_ID;
        if (trs_ID(1)== meas_ID(1) && trs_ID(1) ~= uint16(0) )...
                || (trs_ID(2)== meas_ID(2) && trs_ID(2) ~= uint16(0) ...
                && trs_det_src==meas_det_src)
            IDMatrix(i, j) = 1;
        end
    end
end

end
