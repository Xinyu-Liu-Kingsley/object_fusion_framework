function rawfile = asc2mat(ascfile,dbcfile,dat_path,uifig)
% 读取asc文件中的报文，并保存为mat格式文件（以.canc为扩展名）
%  数据文件中包含名为rawTabs的变量，其保护全部CAN报文

narginchk(2,4)

rawfile = [];

[ascpath, ascname, ascext] = fileparts(ascfile);
assert(strcmp(ascext, '.asc'))
[~, ~, dbcext] = fileparts(dbcfile);
assert(strcmp(dbcext, '.dbc'))
if nargin < 4, uifig = []; end
if nargin < 3, dat_path = ascpath; end

D = dir(ascfile);
if D.bytes > 100*1024*1024
    warndlg('单个asc文件大小超过80M！')
    return
end

if isunix
    warning('Vehicle Network Toolbox仅支持Windows系统!')
    return
end

if ~isempty(uifig), d = uiprogressdlg(uifig, 'Title', 'asc转换成mat...', 'Indeterminate', 'on', 'Cancelable', 'on'); end

% 为保证CANDTU录制文件可用进行预处理，并生成临时asc文件
if ~isempty(uifig), d.Message = '预处理asc文件...'; end
tempfile = 'can.asc';
[~,~] = replaceinfile('\t', ' ', ascfile, tempfile);

% 导入asc文件
if ~isempty(uifig), d.Message = '导入asc文件...'; end
rawTabs0 = canMessageImport(tempfile,'Vector',canDatabase(dbcfile),'OutputFormat','timetable');
delete(tempfile)    % 删除临时asc文件

% 剔除无用报文（如：XCP报文）
if ~isempty(uifig), d.Message = '删除无效报文...'; end
idx = (rawTabs0{:,'ID'} == hex2dec('7EA') | rawTabs0{:,'ID'} == hex2dec('7E8'));
rawTabs0(idx,:) = [];

% asc文件中存在时间重新归零的情况，因此需要分割并单独存储
idx = find(diff(rawTabs0.Time)<-5e2);
sec = [1; idx; height(rawTabs0)]; 

counter = 0;
max_files = 100;
rawfile = cell(1,max_files);
max_items = 2000000;         % 单个mat文件最大报文数量
for i=2:length(sec)
    rawTabs_tmp = rawTabs0(sec(i-1):sec(i),:);
    
    L = size(rawTabs_tmp,1);       % 总报文数量
    
    if ~isempty(uifig), d.Message = sprintf('保存%s格式文件...',AIF.rawext); end
    
    if L <= max_items   % 报文数量不大于最大报文数量值，保存单一文件
        rawTabs = rawTabs_tmp;
        if isempty(idx)
            matname = ascname;
        else
            matname = sprintf('%s_S%d',ascname,i-1);
        end
        counter = counter+1;
        assert(counter <= max_files)
        rawfile{counter} = AIF.save_raw(rawTabs, dat_path, matname);
    else                % 如果大于最大报文数量，则分割后依次保存
        N = ceil(L/max_items);
        for j=1:N
            idx0 = (j-1)*max_items+1;
            if j==N, idx1 = L; else, idx1 = j*max_items; end
            if ~isempty(uifig)
                d.Message = sprintf('保存mat文件...%d/%d', j, N);
            end
            rawTabs = rawTabs_tmp(idx0:idx1,:);

            if isempty(idx)
                matname = sprintf('%s_%d_%d',ascname,idx0,idx1);
            else
                matname = sprintf('%s_S%d_%d_%d',ascname,i-1,idx0,idx1);
            end
            counter = counter+1;
            assert(counter <= max_files)
            rawfile{counter} = AIF.save_raw(rawTabs, dat_path, matname);
        end
    end
end

rawfile = rawfile(1:counter);

if ~isempty(uifig), close(d); end

end

