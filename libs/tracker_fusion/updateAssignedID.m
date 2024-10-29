function [CamID_history,RadID_history] = updateAssignedID(tracks,obs_XY,jdx)
%UPDATEASSINGEDID �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
if tracks.status(jdx).CNT <= 10
    cnt = tracks.status(jdx).CNT;
    tracks.status(jdx).CamID_history(cnt) =  obs_XY.fusedCamID;
    tracks.status(jdx).RadID_history(cnt) =  obs_XY.fusedRadID;
else
    tracks.status(jdx).CamID_history(end) = obs_XY.fusedCamID;
    tracks.status(jdx).CamID_history(1:end-1) = tracks.status(jdx).CamID_history(2:end);
    tracks.status(jdx).RadID_history(end) = obs_XY.fusedRadID;
    tracks.status(jdx).RadID_history(1:end-1) = tracks.status(jdx).RadID_history(2:end);
end
CamID_history = tracks.status(jdx).CamID_history;
RadID_history = tracks.status(jdx).RadID_history;
end

