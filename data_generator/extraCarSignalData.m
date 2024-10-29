function extraCarSignalData(carSignaFile,file,dat_path)
% carSignal（.mat）保存为本地文件（.datCarSignal）

narginchk(2, 3);

% 视频文件路径
[path, name, ~] = fileparts(carSignaFile);

if nargin < 3
    dat_path = path;
end

datCarInfo = fopen(fullfile(dat_path,[name,'.datCarInfo']),'w+');
if datCarInfo ==-1, error('打开%s失败！', fullfile(dat_path, [name, '.datCarInfo'])); end
if ~isempty(file)
    load(file,'car_signals')
    [n,~]= size(car_signals);
    idx_start = 1;
    for i = idx_start:n
        %% ====================carSignals=================================
        temp = [car_signals{i,1},car_signals{i,2},double(car_signals{i,3})];
        fwrite(datCarInfo,int32([i,temp]* aux_const.factor4storing), 'int32');
    end
    fclose(datCarInfo);
    
end







