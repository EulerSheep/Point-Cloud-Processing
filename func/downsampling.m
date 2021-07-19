function [srcd] = downsampling(src,cellsize)
%% 参看 https://blog.csdn.net/shixin_0125/article/details/105183628
%% 改了一下，不在用体素格的第一个点代表整个体素，而是采用质心代替

%看点云在3个坐标轴的最大最小坐标
xlimit = zeros(2,1);
ylimit = zeros(2,1);
zlimit = zeros(2,1);

xlimit(1) = min(src(:,1));
xlimit(2) = max(src(:,1));
ylimit(1) = min(src(:,2));
ylimit(2) = max(src(:,2));
zlimit(1) = min(src(:,3));
zlimit(2) = max(src(:,3));

%%cellsize = 格子的大小，越大，保存下来的点就越少
%统计在3个坐标轴各自需要多少个格子， xmax - xmin  / size 就是，我们顺便向上取值，保证点云的每个点都有
x = floor((xlimit(2) - xlimit(1))/cellsize)+1;
y = floor((ylimit(2) - ylimit(1))/cellsize)+1;
z = floor((zlimit(2) - zlimit(1))/cellsize)+1;

voxel = cell(x,y,z);

%向网格里填数
for i =1:length(src)
    %不加1 可能出现0的情况
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

srcd = points;

end

