function out = LoadControllerDump(DumpFolder, file, varargin)

if (DumpFolder(end) ~= '\')
    DumpFolder = [DumpFolder '\'];
end

DumpFile = [DumpFolder, file];
if (isempty(file))
    DumpFile = [DumpFolder, getlatestfile([DumpFolder '*_controller.csv'])];
end

offset_provided = false;
if (length(varargin) >= 1) % extraction time offset
    tOffset = varargin{1};
    offset_provided = true;    
else
    tOffset = 0;
end

if (length(varargin) >= 2) % extraction duration
    tOffset = varargin{1};        
    tLength = varargin{2};
    offset_provided = true;
else
    tLength = inf;
end

disp(DumpFile)
data = ParseControllerDebug(dlmread(DumpFile, ','));

dataStart = 1;
dataLength = length(data.time);

out = [];
f = fieldnames(data);
for i = 1:length(f)
    if (~isfield(out, f{i}))
        out.(f{i}) = data.(f{i})(dataStart:dataStart+dataLength-1,:);    
    end
end

t0 = out.time(1);
out.time = out.time - t0; % subtract initial timestamp to start from 0

% trim output data
if (offset_provided)
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
    
    % correct output time to match trimmed away time  
    t0 = out.time(1);
    out.time = out.time - t0; % subtract initial timestamp to start from 0    
end

end