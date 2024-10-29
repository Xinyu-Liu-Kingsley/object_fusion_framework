function showImageFusion(ax, frame,stitle)
% 显示图像



cla(ax)
hold(ax, 'on')
image(ax, frame, 'AlphaData', 0.8)
title(ax, stitle, 'Interpreter', 'latex')
set(ax, 'ydir', 'reverse')
axis(ax, 'off')
axis(ax, 'equal')

end



