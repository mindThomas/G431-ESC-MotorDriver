function out = LoadDump(DumpFolder, file, varargin)

if (DumpFolder(end) ~= '/')
    DumpFolder = [DumpFolder '/'];
end

if (isempty(file))
    file = getlatestfile([DumpFolder '*.csv']);
end

if (length(varargin) >= 1) % extraction time offset
    tOffset = varargin{1};
else
    tOffset = 0;
end

if (length(varargin) >= 2) % extraction duration
    tLength = varargin{2};
else
    tLength = inf;
end

DumpFile = [DumpFolder, file];
try
    out = ParseControllerDebug(dlmread(DumpFile, ','));
catch err
    try
        out = ParseCombinedSample(dlmread(DumpFile, ','));
    catch err2
        rethrow(err2);
    end
end

out.time = out.time - out.time(1); % subtract initial timestamp to start from 0

% trim output data
if (length(varargin) > 0)
    outTmp = out;
    
    idxStart = find(outTmp.time >= tOffset); idxStart = idxStart(1);
    idxEnd = find(outTmp.time <= tOffset+tLength); idxEnd = idxEnd(end);
    
    if ((idxStart < 0) || (idxStart > length(outTmp.time)))
        error('Could not trim data: Incorrect start index');
    end
    if ((idxEnd < 0) || (idxEnd > length(outTmp.time)) || (idxEnd < idxStart))
        error('Could not trim data: Incorrect end index');
    end    
    
    out = [];
    f = fieldnames(outTmp);
    for i = 1:length(f)
        out.(f{i}) = outTmp.(f{i})(idxStart:idxEnd,:);    
    end
    
    out.time = out.time - out.time(1); % correct output time to match trimmed away time
end

end