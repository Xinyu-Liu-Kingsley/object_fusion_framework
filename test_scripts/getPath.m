function [raw_path,dat_path,vid_path] = getPath
% 说明：用于读取或存储数据，获取视频，二进制文件等源数据路径；
% 每个人路径可由各自个人ID值获得，以便切换使用者;

    personalID = 2; 
    switch personalID           
        case 1
            if ismac
                raw_path = '/Users/danjiabi/RawData/';      % 源数据路径：.mp4, .mat, .asc
                dat_path = '/Users/danjiabi/LocData/';      % 本地化数据路径：.idxLane, datLane, idxObs, datObs, .canc, .sens
            else
                raw_path = 'D:\RawData\';   	% 元数据路径：.mp4, .mat, .asc
                % 本地化数据路径：.idxLane, datLane, idxObs, datObs, .canc, .sens
                dat_path = 'D:\LocData\';
            end
            vid_path = [raw_path, 'video'];
            % vidfiles = {...
            %     '2021-07-26-14-43-53.mp4';...
            %     '2021-07-26-14-45-54.mp4';...
            %     '2021-07-26-14-46-49.mp4';...
            %     '2021-07-26-14-53-28.mp4';...
            %     '2021-07-26-14-57-57.mp4';...
            %     '2021-07-26-14-59-56.mp4';...
            %     '2021-07-26-15-06-31.mp4';...
            %     '2021-07-26-15-14-58.mp4'...
            %     };
        case 2   % Han
            raw_path = 'U:\摄像头研发测试数据\RawData\20220314\';   	% 源数据路径：.mp4, .mat, .asc
            dat_path = 'U:\摄像头研发测试数据\RawData\20220314\';      % E:\RawDate\vision
            vid_path = [raw_path, 'video'];
            %dat_path = 'E:\RawDate\vision\TSR\2021-04-23-17-44-09'; 
        case 3 % Zhou
%             raw_path = 'D:\Data\Postprocessing\';
%             dat_path = 'D:\Data\Postprocessing\';
            raw_path = 'D:\Data\Postprocessing\CombineData\';
            dat_path = 'D:\Data\Postprocessing\CombineData\';
            vid_path = [raw_path, 'video'];
        otherwise
           error('请输入个人ID，以获取正确的源数据路径。 --getPath'); 
    end
end