function out = ExtractIndices(in, idx)    
    if ((min(idx) < 0) || (max(idx) > length(in.time)))
        error('Could not trim data: Indices out of bounds');
    end

    out = [];
    f = fieldnames(in);
    for i = 1:length(f)
        out.(f{i}) = in.(f{i})(idx,:);    
    end

    out.time = out.time - out.time(1); % correct output time to match trimmed away time
    
end