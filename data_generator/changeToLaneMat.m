% 中转为可供LDW模型仿真使用数据
% load('CLeftMatrix.mat');
% load('CRightMatrix.mat');

frmIdx = 1:900;

L_C0 = CLeftMatrix(frmIdx,5)';
L_C1 = CLeftMatrix(frmIdx,2)';
L_C2 = CLeftMatrix(frmIdx,3)';
L_C3 = CLeftMatrix(frmIdx,4)';

R_C0 = CRightMatrix(frmIdx,5)';
R_C1 = CRightMatrix(frmIdx,2)';
R_C2 = CRightMatrix(frmIdx,3)';
R_C3 = CRightMatrix(frmIdx,4)';

simulinktime = frmIdx;

save('LANE.mat','simulinktime','R_C3','R_C2','R_C1','R_C0' ...
    ,'L_C3','L_C2','L_C1','L_C0');

figure; 
subplot(121);hold on;
plot(CLeftMatrix(frmIdx,1),'.-.');
plot(L_C0,'.-'); 
title('left');
subplot(122);hold on;
plot(CRightMatrix(frmIdx,1),'.-.');
plot(R_C0,'.-'); title('right');

