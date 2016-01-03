
folder = 'PTU Splitter/output/';
files = dir([folder '*.ptu']);

for i=1:1
    
filename = [folder files(i).name]

r = FLIMreaderMex(filename);
n_chan = FLIMreaderMex(r,'GetNumberOfChannels');

t = FLIMreaderMex(r,'GetTimePoints');
data = FLIMreaderMex(r,'GetData',1);

FLIMreaderMex(r,'Delete');
data = sum(data,1);
data = sum(data,2);
data = squeeze(data);
end

clf
imagesc(data')
daspect([1 1 1 ])
%xlim([150 250]);
%ylim([75 160])
colorbar