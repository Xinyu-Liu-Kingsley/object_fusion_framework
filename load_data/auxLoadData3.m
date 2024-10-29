function [lanes_nn1, lanes_nn2, vehs_uv1,vehs_uv2,peds_uv1,peds_uv2,...
    cropSizeObs,cropSizePeds,cropSizeLane,laneCrossSeg,num_frm, carSignals, soft_version,error,video_offset] = auxLoadData3(fullname_without_ext, frmID0, frmID1)    %#codegen
% 从数据文件读取车道线推理结果
% idxLane文件格式。每行由10个int32组成，分别为：帧号、车道线数量、第1车道线点数、...、第8车道线点数
% frameID   LineNum     pts1    pts2    pts3    pts4    pts5    pts6    pts7    pts8
%
% datLane文件格式。每行由8个int32组成，每行对应一个车道线检测点
% [u     v   src   rho     feature1    feature2    feature3    cls] * aux_const.factor4loading
%
% idxObs文件格式。每行由2个int32组成，分别为：帧号、障碍物数量
% frameID   ObsNum
%
% datObs文件格式。每行由9个int32组成，每行对应一个障碍物
% [conf     u1  v1  u2  v2  cu  cv  cls     r] * aux_const.factor4loading

if coder.target('MATLAB')
    if nargin < 3, frmID1 = int32(10); end
    if nargin < 2, frmID0 = int32(1); end
    if nargin < 1
        if ismac, fullname_without_ext = '/Users/danjiabi/RawData/20210727/2021-07-26-14-43-53'; else, fullname_without_ext = 'E:\RawDate\vision\FusionImage\999999_001000'; end
    end
end
%% 检测异常
assert(isa(frmID0, 'int32'))
assert(isscalar(frmID0))
assert(isa(frmID1, 'int32'))
assert(isscalar(frmID1))
assert(frmID0 > 0)
assert(frmID1 > 0)
assert(frmID0 <= frmID1)
%% 帧号处理
if frmID1 > aux_const.max_frames
    disp_info(sprintf('frmID1 reset from %d to %u', frmID1, aux_const.max_frames), info_cls.general, info_spec.warning);
    frmID1 = int32(aux_const.max_frames);
end
frmIDs = frmID0:frmID1;
num_frm = uint32(length(frmIDs));
%% 创建对应结构体
video_offset = 0;
video_offset_threshold = 5;
error = uint8(1);
% 车道线
lanes_nn1 = repmat(lane_const.NN, aux_const.max_frames, 1);
lanes_nn2 = repmat(lane_const.NN, aux_const.max_frames, 1);
coder.cstructname(lanes_nn1,'LaneNN')
coder.cstructname(lanes_nn2,'LaneNN')
cropSizeLane = zeros(aux_const.max_frames,4);
laneCrossSeg = zeros(aux_const.max_frames,2);
% 车辆
vehs_uv1 = repmat(obstacle_const.UV_Vehs, aux_const.max_frames, obstacle_const.max_obs);
coder.cstructname(vehs_uv1, 'ObstacleUV')
vehs_uv2 = vehs_uv1;
% cropSizeLane = zeros(aux_const.max_frames,4);
cropSizeObs = zeros(aux_const.max_frames,4);
% 行人
peds_uv1 = repmat(peds_const.UV_Peds, aux_const.max_frames, peds_const.max_obs);
coder.cstructname(peds_uv1, 'PedsUV')
peds_uv2 = peds_uv1;
cropSizePeds = zeros(aux_const.max_frames,4);
% 遮挡
frameBlock = zeros(aux_const.max_frames,1,'uint8');
% 车速信息
carSignals = zeros(aux_const.max_frames,3);
%% 读取数据
% 版本号路径
datHeader = [fullname_without_ext, '.datHeader'];

% 车道线路径
datLane = [fullname_without_ext, '.datLane'];
idxLane = [fullname_without_ext, '.idxLane'];
% 障碍物路径
datObs = [fullname_without_ext, '.datObs'];
idxObs = [fullname_without_ext, '.idxObs'];
% 行人路径
datPeds = [fullname_without_ext,'.datPeds'];
idxPeds = [fullname_without_ext,'.idxPeds'];
% 裁剪框中的障碍物  斑马线分割 路径
cropSizeAndSeg = [fullname_without_ext, '.cropSizeAndSeg'];
cropSizeObs_ = [fullname_without_ext, '.cropSizeObs'];
% 车速信息
carSignals_ = [fullname_without_ext,'.datCarInfo'];
% 遮挡路径
% datBlock = [fullname_without_ext,'.datBlock'];
% idxBlock = [fullname_without_ext,'.idxBlock'];
% 打开版本号文件
datHeaderID = fopen(datHeader, 'r');
% 打开车道线文件
datLaneID = fopen(datLane, 'r');
idxLaneID = fopen(idxLane, 'r');
% 打开障碍物文件
datObsID = fopen(datObs, 'r');
idxObsID = fopen(idxObs, 'r');
% 打开行人文件
datPedsID = fopen(datPeds, 'r');
idxPedsID = fopen(idxPeds, 'r');
% 打开车速信息
carSignalsID = fopen(carSignals_,'r');
% 打开遮挡文件
% datBlockID = fopen(datBlock, 'r');
% idxBlockID = fopen(idxBlock, 'r');
% 判断版本号是否读取成功
loadVersion = uint8(1);
if datHeaderID == -1
    loadVersion = uint8(0);
end
soft_version = char(0);
%% 判断是打开文件是否成功
% 判断车道线数据是否读取成功
if datLaneID == -1
    fprintf('auxLoadData >> open dat file (%s) failed!\n', datLane)
    return
end
if idxLaneID == -1
    fprintf('auxLoadData >> open idx file (%s) failed!\n', idxLane)
    return
end
% 判断障碍物数据是否读取成功
if datObsID == -1
    fprintf('auxLoadData >> open dat file (%s) failed!\n', datObs)
    return
end
if idxObsID == -1
    fprintf('auxLoadData >> open idx file (%s) failed!\n', idxObs)
    return
end
% 判断行人数据是否读取成功
if datPedsID == -1
    fprintf('auxLoadData >> open dat file (%s) failed!\n', datPeds)
    return
end
if idxPedsID == -1
    fprintf('auxLoadData >> open idx file (%s) failed!\n', idxPeds)
    return
end
% 判断车速信号读取是否成功
% if carSignalsID == -1
%     fprintf('auxLoadData >> open idx carSignals (%s) failed!\n', carSignals_)
%     return
% end

% 判断遮挡数据是否读取成功
% if datBlockID == -1
%     fprintf('auxLoadData >> open dat file (%s) failed!\n', datBlock)
%     return
% end
% if idxBlockID == -1
%     fprintf('auxLoadData >> open idx file (%s) failed!\n', idxBlock)
%     return
% end

% 判断裁剪框中的障碍物 斑马线分割 数据是否读取成功
% if cropSizeAndSegID == -1
%     fprintf('auxLoadData >> open idx file (%s) failed!\n', cropSizeObs_)
%     return
% end

flagsLane = ones(aux_const.max_frames, 1);      % 每帧画面中车道线处理标识
flagsObs = ones(aux_const.max_frames, 1);       % 每帧画面中障碍物处理标识
flagsPeds = ones(aux_const.max_frames,1);       % 每帧画面中行人处理标识
flagsCropAndSeg = ones(aux_const.max_frames, 1);
flagsCarSignals = ones(aux_const.max_frames,1);
% flagsBlock = ones(aux_const.max_frames, 1);
flagsLane([1:frmID0-1, frmID1+1:end]) = 0;
flagsObs([1:frmID0-1, frmID1+1:end]) = 0;
flagsPeds([1:frmID0-1, frmID1+1:end]) = 0;
frm_max = 4501;
flagsCropAndSeg([1:frmID0-1, frm_max+1:end]) = 0;
flagsCarSignals([1:frmID0-1, frmID1+1:end]) = 0;

errorLane = uint8(1);                           % 车道线处理异常标识（全局）
errorObs = uint8(1);                            % 障碍物处理异常标识（全局）
errorPeds = uint8(1);                           % 行人处理异常标识（全局）
% errorBlock = uint8(1);                        % 遮挡处理异常标识（全局）
errorCropAndSeg = uint8(1);                     % 裁剪框障碍物处理异常标识（全局）
errorCarSignals = uint8(1);

if loadVersion == uint8(1)
    soft_version = fread(datHeaderID,'*char')';
end
% 打开裁剪框中的障碍物  斑马线分割 文件
if sum(soft_version) >= sum('1.0.5')
    cropSizeAndSegID = fopen(cropSizeAndSeg, 'r');
else
    cropSizeAndSegID = fopen(cropSizeObs_, 'r');
end

%% 读取车道线数据
cnt = int32(0);     % 帧计数器
frm1 = int32(0);
while ~feof(idxLaneID) && cnt < max(frmIDs)
    idx_line = fread(idxLaneID, double([1, 3]), 'int32');
    cnt = cnt+int32(1);
    if isempty(idx_line)
        fprintf('auxLoadData(lane) >> read warning! cnt : %d\n', cnt)
        break
    end
    assert(cnt == idx_line(1))

    N1 = int32(idx_line(2));
    N2 = int32(idx_line(3));
    lane_nn1 = lane_const.NN;
    lane_nn2 = lane_const.NN;

    if N1 > 0
        lane_nn1.len = N1;
        lane_nn1.src = det_src.overall;
       
        switch soft_version
            case '1.0.0'
                lane_nn1 = protoLane_1_0_0(datLaneID,lane_nn1,N1);
            case {'1.0.1','1.0.2','1.0.3','1.0.4'}
                lane_nn1 = protoLane_1_0_1(datLaneID,lane_nn1,N1);
            case {'1.0.5','1.0.6','1.0.7'}
                lane_nn1 = protoLane_1_0_5(datLaneID,lane_nn1,N1);               
            case '1.0.8'
                lane_nn1 = protoLane_1_0_8(datLaneID,lane_nn1,N1);
            case '2.1.0'
                lane_nn1 = protoLane_2_1_0(datLaneID,lane_nn1,N1);
            otherwise
                lane_nn1 = protoLane_1_0_0(datLaneID,lane_nn1,N1);
        end
       
    end

    if N2 > 0
        lane_nn2.len = N2;
        lane_nn2.src = det_src.partial;

        switch soft_version
            case '1.0.0'
                lane_nn2 = protoLane_1_0_0(datLaneID,lane_nn2,N2);
            case {'1.0.1','1.0.2','1.0.3','1.0.4'}
                lane_nn2 = protoLane_1_0_1(datLaneID,lane_nn2,N2);
            case {'1.0.5','1.0.6','1.0.7'}
                lane_nn2 = protoLane_1_0_5(datLaneID,lane_nn2,N2);               
            case '1.0.8'
                lane_nn2 = protoLane_1_0_8(datLaneID,lane_nn2,N2);
            case '2.1.0'
                lane_nn2 = protoLane_2_1_0(datLaneID,lane_nn2,N2);
            otherwise
                lane_nn2 = protoLane_1_0_0(datLaneID,lane_nn2,N2);
        end
      
    end

    k = find(frmIDs == cnt, 1);
    if ~isempty(k)
        lanes_nn1(cnt) = lane_nn1;
        lanes_nn2(cnt) = lane_nn2;
        frm1 = frm1+int32(1);
    end
    flagsLane(cnt) = 0;

    if sum(flagsLane) == 0
        errorLane = uint8(0);
        break
    end
end
%% 读取障碍物数据
cnt = int32(0);%
frm2 = int32(0);
while ~feof(idxObsID) && cnt < max(frmIDs)
    idx_line = fread(idxObsID, [1, 3], 'int32');
    cnt = cnt+int32(1);
    if isempty(idx_line)
        fprintf('auxLoadData(obs) >> read warning! cnt : %d\n', cnt)
        break
    end
    assert(cnt == idx_line(1))

    k = find(frmIDs == idx_line(1), 1);
    if ~isempty(k)
        frm2 = frm2+int32(1);
    end
    numObs1 = idx_line(2);
    numObs2 = idx_line(3);
    veh_uv1 = obstacle_const.UV_Vehs;
    veh_uv2 = obstacle_const.UV_Vehs;

    if numObs1 > obstacle_const.max_obs
        % warning('obstacle too many! (%d)', numObs)
    end
    if numObs1 > 0        
        if ~isempty(k)
            switch soft_version
                case '2.1.0'
                    vehs_uv1 = protoObj_2_1_0(datObsID,veh_uv1,vehs_uv1,numObs1,cnt);
                otherwise
                    vehs_uv1 = protoObj_1_0_7(datObsID,veh_uv1,vehs_uv1,numObs1,cnt);
            end
        end
    end
    if numObs2 > 0 
        if ~isempty(k)
            switch soft_version
                case '2.1.0'
                    vehs_uv2 = protoObj_2_1_0(datObsID,veh_uv2,vehs_uv2,numObs2,cnt);
                otherwise
                    vehs_uv2 = protoObj_1_0_7(datObsID,veh_uv2,vehs_uv2,numObs2,cnt);
            end
        end
    end

    flagsObs(cnt) = 0;
    if sum(flagsObs) == 0
        errorObs = uint8(0);
        break
    end
end
%% 读取行人数据
cnt = int32(0);%
frm3 = int32(0);
while ~feof(idxPedsID) && cnt < max(frmIDs)
    idx_line = fread(idxPedsID, [1, 3], 'int32');
    cnt = cnt+int32(1);

    if isempty(idx_line)
        fprintf('auxLoadData(obs) >> read warning! cnt : %d\n', cnt)
        break
    end
    assert(cnt == idx_line(1))

    k = find(frmIDs == idx_line(1), 1);
    if ~isempty(k)
        frm3 = frm3+int32(1);
    end
    numObs1 = idx_line(2);
    numObs2 = idx_line(3);
    ped_uv1 = peds_const.UV_Peds;
    ped_uv2 = peds_const.UV_Peds;

    if numObs1 > peds_const.max_obs
        %         warning('obstacle too many! (%d)', numObs)
    end
    if numObs1 > 0   
        if ~isempty(k)
            switch soft_version
                case '2.1.0'
                    peds_uv1 = protoPed_2_1_0(datPedsID,peds_uv1,ped_uv1,numObs1,cnt);
                otherwise
                    peds_uv1 = protoPed_1_0_7(datPedsID,peds_uv1,ped_uv1,numObs1,cnt);
            end
        end
    end
    if numObs2 > 0
        if ~isempty(k)
            switch soft_version
                case '2.1.0'
                    peds_uv2 = protoPed_2_1_0(datPedsID,peds_uv2,ped_uv2,numObs2,cnt);
                otherwise
                    peds_uv2 = protoPed_1_0_7(datPedsID,peds_uv2,ped_uv2,numObs2,cnt);
            end
        end
    end

    flagsPeds(cnt) = 0;
    if sum(flagsPeds) == 0
        errorPeds= uint8(0);
        break
    end
end

%% 读取裁剪框中障碍物数据
cnt = int32(0);
frm5 = int32(0);
video_offset_tmp = zeros(1,2);

while(~feof(cropSizeAndSegID)) %&& cnt < frm_max
    cnt = cnt + int32(1);
    tmp = zeros(1,15);
    if loadVersion == uint8(1) && sum(soft_version) <= sum('1.0.4')
        tmp =(fread(cropSizeAndSegID,[1,5],'int32')) * aux_const.factor4loading;
        if length(tmp) ~= 5
            video_offset_tmp = tmp;
            break
        end
    elseif loadVersion == uint8(1) && sum(soft_version) <= sum('1.0.7')
        tmp =(fread(cropSizeAndSegID,[1,15],'int32')) * aux_const.factor4loading;    
        if length(tmp) ~= 15
            video_offset_tmp = tmp;
            break
        end
    elseif loadVersion == uint8(1) && sum(soft_version) <= sum('2.1.0')
        tmp =(fread(cropSizeAndSegID,[1,15],'int32')) * aux_const.factor4loading;    
    end
    
    if isempty(tmp)
        fprintf('auxLoadData(cropSizeAndSeg) >> read warning! cnt : %d\n', cnt)
        break
    end
    assert(cnt == tmp(1))
    k = 0;
    if 0 < tmp(1) && tmp(1) < aux_const.max_frames
        k = 1;
    end
%     k = find(frmIDs == tmp(1), 1);
    if k>0 
        if loadVersion == uint8(1) && sum(soft_version) <= sum('1.0.4')
            frm5 = frm5+int32(1);
            cropSizeObs(cnt,1:4) = tmp(2:5);
            cropSizePeds(cnt,1:4) = tmp(2:5);
        elseif loadVersion == uint8(1) && sum(soft_version) <= sum('1.0.7')
            frm5 = frm5+int32(1);
            cropSizeObs(cnt,1:4) = tmp(2:5);
            cropSizePeds(cnt,1:4) = tmp(6:9);
            cropSizeLane(cnt,1:4) = tmp(10:13);
            laneCrossSeg(cnt,1:2) = tmp(14:15);
        elseif loadVersion == uint8(1) && sum(soft_version) <= sum('2.1.0')
            frm5 = frm5+int32(1);
            cropSizeObs(cnt,1:4) = tmp(2:5);
            cropSizePeds(cnt,1:4) = tmp(6:9);
            cropSizeLane(cnt,1:4) = tmp(10:13);
            laneCrossSeg(cnt,1:2) = tmp(14:15);
        else 
            cropSizeObs(cnt,1:4) = tmp(2:5);
            frm5 = frm5+int32(1);
        end
    end
    flagsCropAndSeg(cnt) = 0;
    if sum(flagsCropAndSeg) == 0
        errorCropAndSeg = uint8(0);
        break
    end
end
%% 读取帧数偏置
if sum(soft_version) >= sum('1.0.5')  
%     if isempty(video_offset_tmp)
        video_offset = 0;
%     else
%         assert((cnt) == video_offset_tmp(1))
%         video_offset = video_offset_tmp(2);
%     end
else
    video_offset = video_offset_threshold;
end
%% 读取车速信息
cnt = int32(0);
frm7 = int32(0);
if carSignalsID~=-1
    while(~feof(carSignalsID) && cnt < max(frmIDs))
        cnt = cnt + int32(1);
        tmp =(fread(carSignalsID,[1,4],'int32')) * aux_const.factor4loading;
        if isempty(tmp)
            fprintf('auxLoadData(cropSizeLane) >> read warning! cnt : %d\n', cnt)
            break
        end
        assert(cnt == tmp(1))
        k = find(frmIDs == tmp(1), 1);
        if ~isempty(k)
            frm7 = frm7+int32(1);
            carSignals(cnt,1:3) = tmp(2:4);
        end
        flagsCarSignals(cnt) = 0;
        if sum(flagsCarSignals) == 0
            errorCarSignals = uint8(0);
            break
        end
    end
end
%% 读取遮挡数据
% cnt = int32(0);
% frm6 = int32(0);
% while(~feof(datBlockID) && cnt < max(frmIDs))
%     cnt = cnt + int32(1);
%     tmp =(fread(idxBlockID,[1,4],'int32'));
%     if isempty(tmp)
%         fprintf('auxLoadData(frameBlock) >> read warning! cnt : %d\n', cnt)
%         break
%     end
%     assert(cnt == tmp(1))
%     k = find(frmIDs == tmp(1), 1);
%     if ~isempty(k)
%         blockDate = (fread(datBlockID, 1, 'int32')) * aux_const.factor4loading;
%         if loadVersion == uint8(1) && sum(soft_version) >= sum('1.0.3')
%             frm6 = frm6+int32(1);
%             frameBlock(cnt) = uint8(blockDate);
%         else
%             frameBlock(cnt) = uint8(blockDate);
%             frm6 = frm6+int32(1);
%         end
%     end
%     flagsBlock(cnt) = 0;
%     if sum(flagsBlock) == 0
%         errorBlock = uint8(0);
%         break
%     end
% end


fclose(idxLaneID);
fclose(datLaneID);
fclose(idxObsID);
fclose(datObsID);
fclose(idxPedsID);
fclose(datPedsID);
fclose(cropSizeAndSegID);
if carSignalsID~=-1
    fclose(carSignalsID);
end
% fclose(idxBlockID);
% fclose(datBlockID);

error = errorLane + errorObs  + errorCropAndSeg + errorCarSignals;%+ errorBlock;

if errorLane == uint8(0) && errorObs == uint8(0)  && errorPeds == uint8(0)
    disp_info(sprintf('auxLoadData >> read frames from No.%d to No.%d OK! ', frmID0, frmID1), info_cls.general)
else
    k = find(flagsLane ~= 0, 1, 'first');
    if(~isempty(k))
        disp_info(sprintf('auxLoadData(lane) >> read frames from No.%d to No.%d error! No.%d not readed. ( Lane )', ...
            frmID0, frmID1, int32(k(1))), info_cls.general, info_spec.warning)
    end
    k = find(flagsObs ~= 0, 1, 'first');
    if(~isempty(k))
        disp_info(sprintf('auxLoadData(obs) >> read frames from No.%d to No.%d error! No.%d not readed. ( Obstacle )', ...
            frmID0, frmID1, int32(k(1))), info_cls.general, info_spec.warning)
    end
    if frm1 == frm2
        num_frm = uint32(frm1);
        error = uint8(0);
    end
end


end


%% 车道线解析
function lane_nn = protoLane_1_0_0(datLaneID,lane_nn,N)
    for i = 1:N
        points = (fread(datLaneID, [1, 8], 'int32')) * aux_const.factor4loading; % 此处必须执行fread，保证datLane中的每一行数据被顺序读取
        lane_nn.conf(i) = points(1);
        lane_nn.u(i) = points(2);
        lane_nn.v(i) = points(3);
        lane_nn.cls(i) = uint8(points(4));
        lane_nn.theta(i) = points(5);
        lane_nn.feature(i,1:2) = points(6:7);
        lane_nn.h(i) = points(8);
    end
end
function lane_nn = protoLane_1_0_1(datLaneID,lane_nn,N)
    for i = 1:N
        points = (fread(datLaneID, [1, 9], 'int32')) * aux_const.factor4loading; % 此处必须执行fread，保证datLane中的每一行数据被顺序读取            
        lane_nn.conf(i) = points(1);
        lane_nn.u(i) = points(2);
        lane_nn.v(i) = points(3);
        lane_nn.cls(i) = uint8(points(4));
        lane_nn.theta(i) = points(5);
        lane_nn.feature(i,1:2) = points(6:7);
        lane_nn.color(i) = points(8);
        lane_nn.h(i) = points(9);
    end
end
function lane_nn = protoLane_1_0_5(datLaneID,lane_nn,N)
    for i = 1:N
        points = (fread(datLaneID, [1, 10], 'int32')) * aux_const.factor4loading; % 此处必须执行fread，保证datLane中的每一行数据被顺序读取
        lane_nn.conf(i) = points(1);
        lane_nn.u(i) = points(2);
        lane_nn.v(i) = points(3);
        lane_nn.cls(i) = uint8(points(4));
        lane_nn.theta(i) = points(5);
        lane_nn.feature(i,1:2) = points(6:7);
        lane_nn.color(i) = points(8);
        lane_nn.h(i) = points(9);
        lane_nn.seg_conf(i) = points(10);
    end
end
function lane_nn = protoLane_1_0_8(datLaneID,lane_nn,N)
    for i = 1:N
        points = (fread(datLaneID, [1, 11], 'int32')) * aux_const.factor4loading; % 此处必须执行fread，保证datLane中的每一行数据被顺序读取
        lane_nn.conf(i) = points(1);
        lane_nn.u(i) = points(2);
        lane_nn.v(i) = points(3);
        lane_nn.cls(i) = uint8(points(4));
        lane_nn.theta(i) = points(5);
        lane_nn.feature(i,1:2) = points(6:7);
        lane_nn.color(i) = points(8);
        lane_nn.h(i) = points(9);
        lane_nn.seg_conf(i) = points(10);
        lane_nn.driv_area(i) = points(11);
    end
end
function lane_nn = protoLane_2_1_0(datLaneID,lane_nn,N)
    for i = 1:N
        points = (fread(datLaneID, [1, 13], 'int32')) * aux_const.factor4loading; % 此处必须执行fread，保证datLane中的每一行数据被顺序读取
        lane_nn.conf(i) = points(1);
        lane_nn.cls(i) = uint8(points(2));
        lane_nn.u(i) = points(3);
        lane_nn.v(i) = points(4);
        lane_nn.theta(i) = points(5);
        lane_nn.feature(i,:) = points(6:9);
        lane_nn.color(i) = points(10);
        lane_nn.seg_conf(i) = points(11);
        lane_nn.driv_area(i) = points(12);
        lane_nn.clu(i) = points(13);
        lane_nn.h(i) = 0;
    end
end
%% 车辆解析
function vehs_uv = protoObj_2_1_0(datObsID,veh_uv,vehs_uv,numObs,cnt)
    for i = 1:numObs
        tmp = (fread(datObsID, [1, 18], 'int32')) * aux_const.factor4loading;
        veh_uv.conf = tmp(1);
        veh_uv.u = tmp(2);
        veh_uv.v = tmp(3);
        veh_uv.w = tmp(4)-tmp(2);
        veh_uv.h = tmp(5)-tmp(3);
        veh_uv.conf2 =  tmp(7);
        switch tmp(6)
            case 0
                veh_uv.cls = obstacle_cls.car;
            case 1
                veh_uv.cls = obstacle_cls.bus;
            case 2
                veh_uv.cls = obstacle_cls.truck;
            case 3   
                veh_uv.cls = obstacle_cls.tricycle;
            otherwise
                veh_uv.cls = obstacle_cls.unknown;
        end
        %veh_uv1.r = 0;
        veh_uv.r = tmp(13);
        veh_uv.flag = uint8(1);
        veh_uv.u1 = tmp(8);
        veh_uv.v1 = tmp(9);
        veh_uv.w1 = tmp(10)-tmp(8);
        veh_uv.h1 = tmp(11)-tmp(9);
        veh_uv.cls2 = obstacle_cls(tmp(12)+1);
        veh_uv.wheelLine_u1 = tmp(14);
        veh_uv.wheelLine_v1 = tmp(15);
        veh_uv.wheelLine_u2 = tmp(16);
        veh_uv.wheelLine_v2 = tmp(17);
        veh_uv.viewPoint = tmp(18); 
        if i <= obstacle_const.max_obs
            vehs_uv(cnt,i) = veh_uv;
        end
    end
end
function vehs_uv = protoObj_1_0_7(datObsID,veh_uv,vehs_uv,numObs,cnt)
    for i = 1:numObs
        tmp = (fread(datObsID, [1, 18], 'int32')) * aux_const.factor4loading;
        veh_uv.conf = tmp(1);
        veh_uv.u = tmp(2);
        veh_uv.v = tmp(3);
        veh_uv.w = tmp(4)-tmp(2);
        veh_uv.h = tmp(5)-tmp(3);
        veh_uv.conf2 =  tmp(7);
        switch tmp(6)
            case 0
                veh_uv.cls = obstacle_cls.car;
            case 1
                veh_uv.cls = obstacle_cls.bus;
            case 2
                veh_uv.cls = obstacle_cls.truck;
            case 3   
                veh_uv.cls = obstacle_cls.tricycle;
            otherwise
                veh_uv.cls = obstacle_cls.unknown;
        end
        %veh_uv1.r = 0;
        veh_uv.r = tmp(13);
        veh_uv.flag = uint8(1);
        veh_uv.u1 = tmp(8);
        veh_uv.v1 = tmp(9);
        veh_uv.w1 = tmp(10)-tmp(8);
        veh_uv.h1 = tmp(11)-tmp(9);
        veh_uv.cls2 = obstacle_cls(tmp(12)+1);
        veh_uv.wheelLine_u1 = tmp(14);
        veh_uv.wheelLine_v1 = tmp(15);
        veh_uv.wheelLine_u2 = tmp(16);
        veh_uv.wheelLine_v2 = tmp(17);
        veh_uv.viewPoint = tmp(18); 
        if i <= obstacle_const.max_obs
            vehs_uv(cnt,i) = veh_uv;
        end
    end
end
%% 行人解析
function peds_uv = protoPed_1_0_7(datPedsID,peds_uv,ped_uv,numObs,cnt)
    for i = 1:numObs 
        tmp = (fread(datPedsID, [1, 28], 'int32')) * aux_const.factor4loading;
        ped_uv.conf = tmp(1);
        ped_uv.u = tmp(2);
        ped_uv.v = tmp(3);
        ped_uv.w = tmp(4)-tmp(2);
        ped_uv.h = tmp(5)-tmp(3);
        ped_uv.cls = obstacle_cls(tmp(6)+12);
        ped_uv.conf2 = tmp(7);
        ped_uv.flag = uint8(1);
        ped_uv.conf_p1 =tmp(8);
        ped_uv.p1_u = tmp(9);
        ped_uv.p1_v = tmp(10);
        ped_uv.conf_p2 = tmp(11);
        ped_uv.p2_u = tmp(12);
        ped_uv.p2_v = tmp(13);
        ped_uv.conf_p3 = tmp(14);
        ped_uv.p3_u = tmp(15);
        ped_uv.p3_v = tmp(16);
        ped_uv.conf_p4 = tmp(17);
        ped_uv.p4_u = tmp(18);
        ped_uv.p4_v = tmp(19);
        ped_uv.conf_p5 = tmp(20);
        ped_uv.p5_u = tmp(21);
        ped_uv.p5_v = tmp(22);
        ped_uv.conf_p6 = tmp(23);
        ped_uv.p6_u = tmp(24);
        ped_uv.p6_v = tmp(25);
        ped_uv.conf_p7 = tmp(26);
        ped_uv.p7_u = tmp(27);
        ped_uv.p7_v = tmp(28);
        if i <= peds_const.max_obs
            peds_uv(cnt,i) = ped_uv;
        end
    end
end
function peds_uv = protoPed_2_1_0(datPedsID,peds_uv,ped_uv,numObs,cnt)
    for i = 1:numObs 
        tmp = (fread(datPedsID, [1, 6], 'int32')) * aux_const.factor4loading;
        ped_uv.conf = tmp(1);
        ped_uv.u = tmp(2);
        ped_uv.v = tmp(3);
        ped_uv.w = tmp(4)-tmp(2);
        ped_uv.h = tmp(5)-tmp(3);
        ped_uv.cls = obstacle_cls(tmp(6)+12);
        ped_uv.conf2 = 0;
        ped_uv.flag = uint8(1);
        ped_uv.conf_p1 = 0;
        ped_uv.p1_u = 0;
        ped_uv.p1_v = 0;
        ped_uv.conf_p2 = 0;
        ped_uv.p2_u = 0;
        ped_uv.p2_v = 0;
        ped_uv.conf_p3 = 0;
        ped_uv.p3_u = 0;
        ped_uv.p3_v = 0;
        ped_uv.conf_p4 = 0;
        ped_uv.p4_u = 0;
        ped_uv.p4_v = 0;
        ped_uv.conf_p5 = 0;
        ped_uv.p5_u = 0;
        ped_uv.p5_v = 0;
        ped_uv.conf_p6 = 0;
        ped_uv.p6_u = 0;
        ped_uv.p6_v = 0;
        ped_uv.conf_p7 = 0;
        ped_uv.p7_u = 0;
        ped_uv.p7_v = 0;
        if i <= peds_const.max_obs
            peds_uv(cnt,i) = ped_uv;
        end
    end
end