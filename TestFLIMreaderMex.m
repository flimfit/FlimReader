filename = 'TestData\BaseName_9_1.pt3';

r = FLIMreaderMex(filename);
n_chan = FLIMreaderMex(r,'GetNumberOfChannels');

t = FLIMreaderMex(r,'GetTimePoints');
data = FLIMreaderMex(r,'GetData',0);

FLIMreaderMex(r,'Delete');
%%
clf
d = reshape(data,[256,512*512]);
plot(t,sum(d,2))

%%

imagesc(squeeze(sum(data,1)))