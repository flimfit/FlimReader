filename = 'PTU Splitter/output/output_file_0.ptu';

r = FLIMreaderMex(filename);
n_chan = FLIMreaderMex(r,'GetNumberOfChannels');

t = FLIMreaderMex(r,'GetTimePoints');
data = FLIMreaderMex(r,'GetData',0);

FLIMreaderMex(r,'Delete');

data = sum(data,1);
data = squeeze(data);

clf
imagesc(data')
daspect([1 1 1 ])
colorbar