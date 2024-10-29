classdef AIF
% Description: 与ADAS相关的数据交换格式，以及数据文件定义规则
%   ADAS interchange format
% Author :  Shane Liu
% Modify Note: 
% ************************************************************ % 
    properties (Constant)
        rawext = '.canc'    % 原始报文文件扩展名
        rawvar = 'rawTabs'  % 原始报文文件中的变量名称
        
        sensext = '.sens'   % 感知数据文件扩展名
        sensvar = 'sensTab' % 感知数据文件中的变量名称
        
        caliext = '.cali'   % 标定数据文件扩展名
        calivar = 'caliTab' % 标定数据文件中的变量名称
        
        max_tar = uint32(100)   % 单帧最大目标数量
        max_case = uint32(10)   % 单个数据文件最多测试用例数量
        
        tarInfo = struct('ID',0,'x',0,'y',0,'vx',0,'vy',0,'ax',0,'ay',0,'RCS',0,'Prop',0,'cls',0);
        frmInfo = repmat(AIF.tarInfo,1,AIF.max_tar);
        
        cali = struct('file',{},'AEB',{},'Veh',{},'Tar',{},'cases',{});
        
        AEB = struct('tm',{},'ASR1_ON',{},'ASR2_ON',{},'Dis_AEB',{},'Dis_FCW',{},'Dis_EBA',{},'Min_Dis',{});
        Veh = struct('tm',{},'VehSpd',{},'VehAcc',{});
        Tar = struct('tm',{},'RelDistLo',{},'RelDistLa',{},'RelSpd',{},'ObjSpd',{},'ObjAcc',{});
    end
    
    methods (Static)
        
        % 保存原始报文
        function rawfile = save_raw(raw, path, name)
            rawfile = [];
            if ~istimetable(raw)
                warning('输入不是timetable类型变量!')
                return
            end
            eval(sprintf('%s = raw;', AIF.rawvar)) % 按照规定的变量名称重命名
            rawfile = fullfile(path,sprintf('%s%s', name, AIF.rawext));
            eval(sprintf('save %s %s', rawfile, AIF.rawvar))
        end
        
        % 读取原始报文文件
        function raw = load_raw(rawfile)
            raw = [];
            if ~exist(rawfile, 'file')
                warning('%s不存在!', rawfile)
                return
            end
            eval(sprintf('load %s -mat %s', rawfile, AIF.rawvar))
            if exist(AIF.rawvar, 'var')
                eval(sprintf('raw = %s;', AIF.rawvar))
            else
                warning('%s中没有变量%s!', rawfile, AIF.rawvar)
            end
        end
        
        % 保存感知数据
        function sensfile = save_sens(sens, path, name)
            sensfile = [];
            if ~isstruct(sens)
                warning('输入不是结构体数组类型变量!')
                return
            end
            eval(sprintf('%s = sens;', AIF.sensvar)) % 按照规定的变量名称重命名
            sensfile = fullfile(path,sprintf('%s%s', name, AIF.sensext));
            eval(sprintf('save %s %s', sensfile, AIF.sensvar))
        end
        
        % 读取感知数据
        function sens = load_sens(sensfile)
            sens = [];
            if ~exist(sensfile, 'file')
                warning('%s不存在!', sensfile)
                return
            end
            eval(sprintf('load %s -mat %s', sensfile, AIF.sensvar))
            if exist(AIF.sensvar, 'var')
                eval(sprintf('sens = %s;', AIF.sensvar))
            else
                warning('%s中没有变量%s!', sensfile, AIF.sensvar)
            end
        end
        
        % 保存标定数据
        function califile = save_cali(cali, path, name)
            califile = [];
            if ~isstruct(cali)
                warning('输入不是结构体数组类型变量!')
                return
            end
            eval(sprintf('%s = cali;', AIF.calivar)) % 按照规定的变量名称重命名
            califile = fullfile(path,sprintf('%s%s', name, AIF.caliext));
            eval(sprintf('save %s %s', califile, AIF.calivar))
        end
        
        % 读取原始报文文件
        function cali = load_cali(califile)
            cali = [];
            if ~exist(califile, 'file')
                warning('%s不存在!', califile)
                return
            end
            eval(sprintf('load %s -mat %s', califile, AIF.calivar))
            if exist(AIF.calivar, 'var')
                eval(sprintf('cali = %s;', AIF.calivar))
            else
                warning('%s中没有变量%s!', califile, AIF.calivar)
            end
        end
        
        % 从某一特定ID的报文列表ttMsg中提取信息，并生成信号列表ttSig
        %   ttMsg - 从asc文件中提取的某一特定ID报文，timetable
        %   field_m - 报文中的信号名称, cell数组
        %   field_s - 信号列表中的字段, cell数组，长度与field_m一致
        %   d - 进度对话框实例
        function ttSig = msg2sig(ttMsg, field_m, field_s, d)
            ttSig = [];
            tmpSig = struct();
            if isempty(ttMsg), return; end
            if ~iscell(field_m) || ~iscell(field_s), return; end
            assert(length(field_m) == length(field_s))
            
            for i=1:height(ttMsg)
                tmpSig(i).tm = ttMsg.Time(i);
                for j=1:length(field_m)
                    eval(sprintf('tmpSig(i).%s = ttMsg{i,''Signals''}{1}.%s;', field_s{j}, field_m{j}))
                end
                if (~isempty(d) && d.CancelRequested), close(d); return; end
            end
            ttSig = struct2table(tmpSig);
        end
    end
    
end

