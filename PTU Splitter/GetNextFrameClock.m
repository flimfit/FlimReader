function pos = GetNextFrameClock(fid)

    n = 2000;

    fseek(fid, 4, 'cof'); % make sure we don't read last frame clock
    pos = ftell(fid);
    found = false;
    num_line = 0;
    
    while (~feof(fid))
        [T3Record, count] = fread(fid, n, 'ubit32'); 
        chan = bitand(bitshift(T3Record,-28),15); 
        markers = bitand(bitshift(T3Record,-16),15);
        isframe = (chan == 15) & (bitand(markers, 4) > 0);
        
        idx = find(isframe,1);
        
        if (~isempty(idx))
            found = true;
            pos = pos + idx * 4;
            num_line = num_line + sum((chan(1:idx) == 15) & (bitand(markers(1:idx), 1) > 0));
            break;
        else
            num_line = num_line + sum((chan == 15) & (bitand(markers, 1) > 0));
            pos = pos + count * 4;
        end
        
        
    end
    
    if (found)
        fseek(fid, pos, 'bof');
    end
    
    disp(['Num Lines ' num2str(num_line)])
end