function dataArray = getDataFromEclipseMemoryTextDump(file)
    dataArray = [];
    f = fopen(file);

    while(true)
        line = fgetl(f);
        if ~ischar(line)
            break
        end
        if (length(strfind(line, 'Details:{')))
            C = regexp(line, '([-0-9]+)', 'match');
            for (i = 1:length(C))
                dataArray(end+1) = str2num(C{i});
            end        
        end
    end
    
    fclose(f);

    %textscan(f, 'Details:{%s}%[^\n]', 'MultipleDelimsAsOne',true)
