function [x_error_history,y_error_history] = updateAssignedError(tracks,obs_XY,jdx)
%UPDATEASSINGEDID
%  更新横纵向关联误差
cnt = tracks.status(jdx).CNTRad;
if cnt>=1
    if tracks.status(jdx).CNTRad <= 6
        tracks.status(jdx).x_error_history(cnt) =  obs_XY.x_error;
        tracks.status(jdx).y_error_history(cnt) =  obs_XY.y_error;
    else
        tracks.status(jdx).x_error_history(end) = obs_XY.x_error;
        tracks.status(jdx).x_error_history(1:end-1) = tracks.status(jdx).x_error_history(2:end);
        tracks.status(jdx).y_error_history(end) = obs_XY.y_error;
        tracks.status(jdx).y_error_history(1:end-1) = tracks.status(jdx).y_error_history(2:end);
    end
end
x_error_history = tracks.status(jdx).x_error_history;
y_error_history = tracks.status(jdx).y_error_history;
end



