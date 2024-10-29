function [cRadar] = hostIdRadar(cRadar,frmidx)
% Description: 
% Author :  Shane Liu
% Modify Note: 
% ************************************************************ % 
%  [time id x,y,vx,vy,RCS]
data = cRadar.data;
idVector = unique(data(1:frmidx,2));
idThres = 30; % 2/0.06
for j = 1:numel(idVector)
    if idVector(j)~=0 
         idxj = (data(1:frmidx,2)==idVector(j));
         if sum(idxj)<idThres
             data(idxj,2:7) = 0;
         end
    end
end

cRadar.data = data;

end