   
%clear all;
pathstr = {'I:/Simulator/final_data_3.npy','I:/Simulator/groundtruth_1.npy','I:/Simulator/groundtruth_2.npy'};
            

close all;
ranges = [];
names = {};
for d = 1:length(pathstr)
    tmp = cat(1, ranges, readNPY(pathstr{d}));
%     groundtruth = squeeze(tmp(:,1,1:1080));
    ranges = squeeze(tmp(:,1:1080));
    %ranges2 = squeeze(tmp(:,1:1080));
    %ranges3 = squeeze(tmp(:,1:1080));
    [~, names{d} ,~]= fileparts(fileparts(pathstr{d}));
end



%{
close all;
ranges = [];
names = {};
for d = 1:length(pathstr)
    tmp = cat(1, ranges, readNPY(pathstr{d}));
%     groundtruth = squeeze(tmp(:,1,1:1080)); %changed 1081
    ranges = squeeze(tmp(:,1:1080));
    [~, names{d} ,~]= fileparts(fileparts(pathstr{d}));
end
%}

g_size = size(ranges);
g_hists = [];
%g_hists2 = [];
%g_hists3 = [];
edges = linspace(0,30,600);
angles = linspace(-pi*0.75, pi*0.75, 1080); %changed the angles from 1081
for i=1:g_size(2)
    [N,new_len] = histcounts(ranges(:,i), edges);
    g_hists = [g_hists; N];
    %g_hists2 = [g_hists2; N];
    %g_hists3 = [g_hists3; N];
end
figure('Name',pathstr{d}); %,'Position',[0 0 800 600]

surf(edges(1:end-1),angles,g_hists,'EdgeColor','none');
%g_histnew = g_hist;
colors = colormap;
view(0, 90);
ylim([-pi*0.75, pi*0.75]);
ylabel('angle [rad]');
xlabel('range [m]');
datfilename = strjoin(names,'_');
datfilename = strrep(datfilename,'_','');
xvalues = edges(1:end-1) + diff(edges);
yvalues = angles;


colorchoise = uint32(g_hists/max(g_hists(:)) *63)+1;
colorhist = reshape(colors(colorchoise(:),:),[1080, 599, 3]);%size changed from 1081
figure;
imshow(colorhist);
imwrite(colorhist,strcat('home/ga83hin/Thesis/laser-calibration/data_sets/Merge_data/temp_storage/',datfilename,'.png'))

colorsout = flipud(reshape(colors, [64,1,3]));
imshow(colorsout);
imwrite(colorsout,strcat('home/ga83hin/Thesis/laser-calibration/data_sets/Merge_data/temp_storage/histcolors.png'))



    