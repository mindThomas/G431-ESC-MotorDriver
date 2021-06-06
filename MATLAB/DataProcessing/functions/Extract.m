function out = Extract(in, idxStart, varargin)
    if (length(varargin) > 0)
        idxEnd = varargin{1};
    else
        idxEnd = length(in.time);
    end

    if ((idxStart < 0) || (idxStart > length(in.time)))
        error('Could not trim data: Incorrect start index');
    end
    if ((idxEnd < 0) || (idxEnd > length(in.time)) || (idxEnd < idxStart))
        error('Could not trim data: Incorrect end index');
    end    

    out = [];
    f = fieldnames(in);
    for i = 1:length(f)
        out.(f{i}) = in.(f{i})(idxStart:idxEnd,:);    
    end

    out.time = out.time - out.time(1); % correct output time to match trimmed away time
    
end