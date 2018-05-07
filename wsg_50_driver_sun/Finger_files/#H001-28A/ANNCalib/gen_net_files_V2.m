clear all,close all,clc

system('rm *.txt');

load net25_fs50

numLayers = net.numLayers;

sizeLayers = zeros(numLayers,1);

for jj = 0:(numLayers-1)
    
    if(jj==0)
       W = net.IW{1};
       dimInput = size(W,2);
    else
       W = net.LW{jj+1,jj}; 
    end
    
    b = net.b{jj+1};
    
    sizeLayers(jj+1) = length(b);
    
    eval(['save -ascii -double W' num2str(jj) '.txt W'])
    eval(['save -ascii -double b' num2str(jj) '.txt b'])
    
end



mMi = double([settings_i.xmin settings_i.xmax]);
save -ascii -double mM_i.txt mMi

mMo = double([settings_t.xmin settings_t.xmax]);
save -ascii -double mM_o.txt mMo

fileID = fopen('meta.txt','w');
fprintf(fileID,'%d\n', numLayers);
fprintf(fileID,'%d\n', dimInput);
fclose(fileID);

fileID = fopen('sizeLayers.txt','w');
for ii = 1:numLayers
    fprintf(fileID,'%d\n', sizeLayers(ii));
end

fclose(fileID);



