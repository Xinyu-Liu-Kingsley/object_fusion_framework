function radfile = parsing_ars408(rawfile, dat_path, uifig)
%PARSING_ARS408 此处显示有关此函数的摘要
%   此处显示详细说明

narginchk(1,3)
if nargin < 3, uifig = []; end

rawTabs = AIF.load_raw(rawfile);

[path, name, ~] = fileparts(rawfile);
if nargin < 2, dat_path = path; end

radPackage = AIF.frmInfo;
radTimestamp = 0;
initFlg = 0;            % 初始标识。标识是否接收过60A报文
numObj = 0;
counterB = 0;
counterD = 0;
counter = 0;            % 60A计数器
radArray = struct('tm', {}, 'frame', {});

L = size(rawTabs,1);    % 报文数量

if ~isempty(uifig)
    d = uiprogressdlg(uifig, 'Title', '解析ARS408报文', 'Cancelable', 'on');
end

for i=1:L
    if rawTabs.ID(i) == hex2dec('60A')
        if initFlg && (counterB ~= numObj || counterD ~= numObj)
            warning('radar lost some frames! %.4f sec %d %d, %d', seconds(rawTabs.Time(i)), numObj, counterB, counterD)
        elseif initFlg
            T = struct2table(radPackage);
            dist = [T.y T.x];
            idx = ~any(dist,2);     % 标记dist中两值均为零的行
            radPackage(idx) = [];   % 剔除空白行
            counter = counter+1;
            radArray(counter) = struct('tm', radTimestamp, 'frame', radPackage);
        end
        if initFlg == 0, initFlg = 1; end 
        
        if ~isempty(uifig)
            if d.CancelRequested, break; end
            d.Value = i/L;
            d.Message = sprintf('%d/%d...', i, L);
        end
        
        numObj = rawTabs{i,'Signals'}{1}.Object_NofObjects;
        radTimestamp = seconds(rawTabs.Time(i));
        counterB = 0;
        counterD = 0;
        radPackage = AIF.frmInfo;
    elseif rawTabs.ID(i) == hex2dec('60B')
        counterB = counterB+1;

        id = uint32(rawTabs{i,'Signals'}{1}.Object_ID+1);
        radPackage(id).ID = id;
        radPackage(id).x = rawTabs{i,'Signals'}{1}.Object_DistLong;
        radPackage(id).y = rawTabs{i,'Signals'}{1}.Object_DistLat;
        radPackage(id).vx = rawTabs{i,'Signals'}{1}.Object_VrelLong;
        radPackage(id).vy = rawTabs{i,'Signals'}{1}.Object_VrelLat;
        radPackage(id).RCS = rawTabs{i,'Signals'}{1}.Object_RCS;
        radPackage(id).Prop = rawTabs{i,'Signals'}{1}.Object_DynProp;
    elseif rawTabs.ID(i) == hex2dec('60D')
        counterD = counterD+1;

        id = uint32(rawTabs{i,'Signals'}{1}.Object_ID+1);
        if radPackage(id).ID ~= id
            warning('radar frames errors!')
        end
        radPackage(id).ax = rawTabs{i,'Signals'}{1}.Object_ArelLong;
        radPackage(id).ay = rawTabs{i,'Signals'}{1}.Object_ArelLat;
        radPackage(id).cls = rawTabs{i,'Signals'}{1}.Object_Class;
    end
end

radfile = AIF.save_sens(radArray, dat_path, name);

if ~isempty(uifig)
    close(d);
end

end

