
folder = 'PTU Splitter/output/';
files = dir([folder '*.ptu']);

for i=1:100
    
filename = [folder files(i).name]

r = FLIMreaderMex(filename);
n_chan = FLIMreaderMex(r,'GetNumberOfChannels');

t = FLIMreaderMex(r,'GetTimePoints');
data = FLIMreaderMex(r,'GetData',[1]);

FLIMreaderMex(r,'Delete');
data = sum(data,1);
data = sum(data,2);
data = squeeze(data);
end

clf
imagesc(data)
caxis([0 100])
daspect([1 1 1 ])
colorbar