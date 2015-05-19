function SplitPTU

    global pathname

    [filename,pathname] = uigetfile('*.ptu','Choose File',pathname);


    prompt = {'Autofocus frames: ';'Frame averaging: ';'Line averaging: '};
    default = {'20', '64', '3'};
    answer = inputdlg(prompt,'Splitting Options',1,default);
    n_skip = str2double(answer{1});
    n_frame = str2double(answer{2});
    line_averaging = str2double(answer{3});


    fid=fopen([pathname filename]);

    header_size = GetHeaderSize(fid);
    fseek(fid, 0, 'bof');

    end_tag_size = 48;
    header_data = fread(fid, header_size - end_tag_size);
    fseek(fid, header_size, 'bof');

    idx = 0;
    n = 0;

    start_pos = GetNextFrameClock(fid);
    finished = false;

    while (~feof(fid))

        fname = ['output' filesep 'output_file_' num2str(idx) '.ptu'];
        ofid = fopen(fname,'w');

        fwrite(ofid, header_data); 
        InsertIntegerTag(ofid, 'Line_Averaging', line_averaging);
        InsertEndTag(ofid);
        
        
        for i=1:n_skip
            start_pos = GetNextFrameClock(fid);
            n = n + 1;
            disp(['Skipping (' num2str(i) ')']);
        end
        for i=1:n_frame
            pos = GetNextFrameClock(fid);
            n = n + 1;
            disp(['Reading (' num2str(i) ') from : ' num2str(start_pos) ' to ' num2str(pos)]);
            fseek(fid, start_pos, 'bof');
            data = fread(fid, pos - start_pos);

            if start_pos == pos
                finished = true;
                break
            end

            start_pos = pos;
            fwrite(ofid, data);
        end

        if (finished)
            break;
        end

        fclose(ofid);
        idx = idx + 1;
    end

    fclose(fid);

    function InsertIntegerTag(fid, ident, value)
        tyInt8 = hex2dec('10000008');
        ident = char(ident);
        padding = repmat(char(0),1,32-length(ident)); 
        fwrite(fid, [ident padding]);
        
        fwrite(fid, 0, 'int32');
        fwrite(fid, tyInt8, 'uint32');
        fwrite(fid, value, 'int64');        
    end
    
    function InsertEndTag(fid)
        tyEmpty8 = hex2dec('FFFF0008');
        ident = 'Header_End';
        padding = repmat(char(0),1,32-length(ident)); 
        fwrite(fid, [ident padding]);
        fwrite(fid, 0, 'int32');
        fwrite(fid, tyEmpty8, 'uint32');
    end


end