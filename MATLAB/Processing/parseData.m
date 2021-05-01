function [out, offset] = parseData(dataArray, offset, type, length)
    eval(['tmp = ' type '(0);']);
    s = whos('tmp');
    size = s.bytes;
    out = typecast(dataArray(offset:(offset+size*length-1)), type);
    offset = offset + size*length;
end