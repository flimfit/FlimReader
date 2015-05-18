global pathname

[filename,pathname] = uigetfile('*.ptu','Choose File',pathname);



%%
prompt = {'Frame averaging: '};
default = {'64'};
answer = inputdlg(prompt,'Splitting Options',1,default);
n_frame = str2double(answer{1});


fid=fopen([pathname filename]);

header_size = GetHeaderSize(fid);
fseek(fid, 0, 'bof');

header_data = fread(fid, header_size);

% Ignore data before first frame clock
%{
n = 0;
while ~feof(fid)
    start_pos = GetNextFrameClock(fid);
    n = n + 1
end
n
%}

%%
idx = 0;

n = 0;

start_pos = GetNextFrameClock(fid);
finished = false;

while (~feof(fid))

    fname = ['output' filesep 'output_file_' num2str(idx) '.ptu'];
    ofid = fopen(fname,'w');
    
    fwrite(ofid, header_data);    
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
    
    fclose(ofid);
    idx = idx + 1;
end

fclose(fid)
