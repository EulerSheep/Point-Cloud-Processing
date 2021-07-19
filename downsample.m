clc;
clear;
close all;

pc = pcread('./data/src.pcd');
src = pc.Location;

srcd = downsampling(src,0.05);

%看点云在3个坐标轴的最大最小坐标
xlimit = pc.XLimits;
ylimit = pc.YLimits;
zlimit = pc.ZLimits;
%%格子的大小，越大，保存下来的点就越少
cellsize = 0.005;

%统计在3个坐标轴各自需要多少个格子， xmax - xmin  / size 就是，我们顺便向上取值ceil
x = ceil((xlimit(2) - xlimit(1))/cellsize);
y = ceil((ylimit(2) - ylimit(1))/cellsize);
z = ceil((zlimit(2) - zlimit(1))/cellsize);

voxel = cell(x,y,z);

%向网格里填数
for i =1:length(src)
    I = floor((src(i,1)-xlimit(1))/cellsize)+1;
    J = floor((src(i,2)-ylimit(1))/cellsize)+1;
    K = floor((src(i,3)-zlimit(1))/cellsize)+1;
    voxel{I,J,K} = [voxel{I,J,K};src(i,:)];
end

%以网格中第一个点对原点云进行下采样
points =[];
for i=1:x
    for j=1:y
        for k=1:z
            if isempty(voxel{i,j,k})==0
                %用整个体素的重心代表这个格子
                point = mean(voxel{i,j,k},1);
                points=[points;point];
            end
        end
    end
end


figure;
pcshow(points,'red');

