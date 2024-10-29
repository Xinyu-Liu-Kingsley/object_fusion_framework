close all
clear
clc


% 获取本地数据路径
get_local_path_const();

% 创建本地化数据路径
if ~exist(dat_path, 'dir')
    [~,~,~] = mkdir(dat_path);
end

% 列举全部视频文件
files = dir(fullfile(raw_path, 'bin', '*.bin'));

% 输入数据来源
sourceDir = input('请输入mat数据来源（0模型推理，1板间存储bin文件解析）：\n');

% 遍历元数据中的视频文件
for i= 1:length(files)
    if ~files(i).isdir
        fprintf('处理%s，%d - %d ...', files(i).name, i, length(files))
        [~, name, ~] = fileparts(files(i).name);    % 提取文件名。一般以时间戳命名
        file_name = fullfile(dat_path,'mat',[name, '.mat']); % 存储模型推理结果
        car_signals_name = fullfile(dat_path,'mat',[name, '_car_signals.mat']);
        if ~exist(file_name,'file')
            warning('no file! (%s)', file_name)
            file_name = [];
        end
        % 创建神经网络检测数据本地化文件
        if ~isempty(file_name)
            try
                if sourceDir == 1
                    extraStoringMatData(fullfile(raw_path, 'bin', [name, '.bin']),file_name,dat_path);
                    extraCarSignalData(fullfile(raw_path, 'bin', [name, '.bin']),car_signals_name,dat_path);
                else
                    extraStoringMatDataModel(fullfile(raw_path, 'video', [name, '.mp4']),file_name,dat_path);
                end
            catch
                continue;
            end
        end

        % 雷达、整车数据
        if ispc
            radar_file = fullfile(raw_path, 'mat', [name, '_cans.mat']);
            bin_file = fullfile(raw_path, 'bin', [name, '.bin']);
            if exist(radar_file, 'file')
                try
                    radfile = radar408CANParsing(radar_file,bin_file,dat_path);
                catch
                    continue;
                end
            else
                warning('no radar mat file! (%s)', radar_file)
            end

        else
            canc_file = fullfile(raw_path, 'canc', [name, '.canc']);
            if exist(canc_file, 'file')
                % 从Matlab数据文件中提取泛化的感知数据文件
                radfile = parsing_ars408(fullfile(raw_path, 'canc', [name, '.canc']), dat_path);
            else
                warning('no canc file! (%s)', canc_file)
            end
        end
        fprintf('OK!\n')
    end
end
