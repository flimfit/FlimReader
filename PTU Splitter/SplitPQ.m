function SplitPQ(pathname,filename)

    global ptu_pathname__;
    global ptu_default__;
    
    ptu_pathname__ = '';
    ptu_default__ = {'0', '20', '4'};
    
    if nargin == 0    
        [filename,pathname] = uigetfile({'*.ptu','*.pt3'},'Choose File',ptu_pathname__);
        ptu_pathname__ = pathname;
        
        prompt = {'Autofocus frames: ';'Frame averaging: ';'Line averaging: '};
        answer = inputdlg(prompt,'Splitting Options',1,ptu_default__);
        ptu_default__ = answer;
    else
        answer = ptu_default__;
    end
    
    [~, output_prefix, ext] = fileparts(filename);
    
    output_dir = [pathname output_prefix filesep];    
    mkdir(output_dir);
    
    n_skip = str2double(answer{1});
    n_frame = str2double(answer{2});
    line_averaging = str2double(answer{3});


    fid=fopen([pathname filename]);

    [header_size, end_tag_size] = GetHeaderSize(fid, ext);
    fseek(fid, 0, 'bof');

    header_data = fread(fid, header_size - end_tag_size);
    
    fseek(fid, header_size-4, 'bof');

    idx = 0;
    n = 0;

    finished = false;

    while (~feof(fid))

        fname = [output_dir output_prefix '_' num2str(idx) ext];
        ofid = fopen(fname,'w');

        fwrite(ofid, header_data); 
        
        if strcmp(ext,'.ptu')
            InsertIntegerTag(ofid, 'Line_Averaging', line_averaging);
            InsertEndTag(ofid);
        end
        
        start_pos = ftell(fid);
        for i=1:n_skip
            pos = GetNextFrameClock(fid);
            disp(['Skipping (' num2str(i) ') : ' num2str(pos - start_pos)]);
            start_pos = pos;
            n = n + 1;
        end
        for i=1:n_frame
            pos = GetNextFrameClock(fid);
            n = n + 1;
            disp(['Reading (' num2str(i) ')  : ' num2str(pos - start_pos)]);
            fseek(fid, start_pos, 'bof');
            data = fread(fid, pos - start_pos);

            if start_pos == pos
                finished = true;
                break
            end

            start_pos = pos;
            fwrite(ofid, data);
        end
        
        %start_pos = GetNextFrameClock(fid);
        %fseek(fid, start_pos, 'bof');
       
        if (finished)
            break;
        end

        fclose(ofid);
        idx = idx + 1;
    end

    fclose(ofid);
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
        fwrite(fid, 0, 'int64');        
    end


end