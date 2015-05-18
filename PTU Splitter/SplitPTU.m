filename = 'C:\Users\sean\Desktop\Test Picoharp Data\data frames=3 lineacc=2 facc=64.ptu';
fid=fopen(filename);

header_size = GetHeaderSize(fid);
fseek(fid, 0, 'bof');

header_data = fread(fid, header_size);

% Ignore data before first frame clock
n = 0;
while ~feof(fid)
    start_pos = GetNextFrameClock(fid);
    n = n + 1
end
n


idx = 0;
n_frame = 64;

n = 0;

start_pos = GetNextFrameClock(fid);

while (~feof(fid))

    fname = ['C:\Users\sean\Desktop\Test Picoharp Data\output_file_' num2str(idx) '.ptu'];
    ofid = fopen(fname,'w');
    
    fwrite(ofid, header_data);    
    for i=1:n_frame
        pos = GetNextFrameClock(fid);
        n = n + 1;
        disp(['Reading (' num2str(i) ') from : ' num2str(start_pos) ' to ' num2str(pos)]);
        fseek(fid, start_pos, 'bof');
        data = fread(fid, pos - start_pos);
        
        if start_pos == pos
            break
        end
        
        start_pos = pos;
        fwrite(ofid, data);
    end
    
    fclose(ofid);
    idx = idx + 1;
end

