function export2capsule(srcPath,desPath,utils,zipflag)
%EXPORT2CAPSULE 此处显示有关此函数的摘要
%   此处显示详细说明

if nargin < 4
    zipflag = 0;
end

% 判断是否正常生成代码
if ~exist(srcPath, 'dir')
    error('source folder (%s) dose not exist!', srcPath)
end

% 判断压缩源码存放路径是否存在
zipDesPath = [desPath,'packages/'];
if ~exist(zipDesPath, 'dir')
    [~,~,~] = mkdir(zipDesPath);
end

% 判断源码存放路径是否存在
srcDesPath = [desPath,'codegen/',utils,'/'];
if exist(srcDesPath, 'dir')
    [~,~,~] = rmdir(srcDesPath, 's');
end
[~,~,~] = mkdir([srcDesPath, 'examples/']);

% 拷贝源码
[~,~,~] = copyfile([srcPath,'*.h'], srcDesPath);
[~,~,~] = copyfile([srcPath,'*.c'], srcDesPath);
[~,~,~] = copyfile([srcPath,'*.cpp'], srcDesPath);
[~,~,~] = copyfile([srcPath,'examples/*.h'], [srcDesPath, 'examples/']);
[~,~,~] = copyfile([srcPath,'examples/*.c'], [srcDesPath, 'examples/']);
[~,~,~] = copyfile([srcPath,'examples/*.cpp'], [srcDesPath, 'examples/']);

% 压缩源码
if zipflag
    zip([zipDesPath, utils, '.zip'], {'*.h', '*.c', '*.cpp', 'examples/*.h', 'examples/*.c', 'examples/*.cpp'}, srcPath)
end

end

