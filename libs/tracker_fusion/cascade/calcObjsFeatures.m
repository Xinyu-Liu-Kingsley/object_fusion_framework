% Description: 计算距离相似度
% Author: Zhou Yanozng
% log:
% 20230727 : created
%****************************************************************%
function feature = calcObjsFeatures(tracks_cas, dets_cas, numM, numT, featureW, lateralDistThres)
feature = ones(numM, numT, 'single')*10000;

for ii = 1:numT
    if (tracks_cas.isUse(ii) >= 1)
        x1  = tracks_cas.x(ii);
        y1  = tracks_cas.y(ii);
        vx1  = tracks_cas.vx(ii);
        vy1  = tracks_cas.vy(ii);
        trs_fea = single([x1, y1, vx1, vy1].*featureW);
        for jj = 1:numM
            if (dets_cas.isUse(jj) >= 1)
                x2  = dets_cas.x(jj);
                y2  = dets_cas.y(jj);
                vx2  = dets_cas.vx(jj);
                vy2 = dets_cas.vy(jj);
                meas_fea = single([x2, y2, vx2, vy2].*featureW);
                if abs(x1 - x2) < lateralDistThres %横向距离限制
                    feature(jj, ii) =norm(meas_fea-trs_fea); %余弦相似度
                end
            end
        end
    end
end

end

