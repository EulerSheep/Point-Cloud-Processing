% icp 算法实现
% 原理参考 https://zhuanlan.zhihu.com/p/104735380
% 代码参考 https://blog.csdn.net/dayuhaitang1/article/details/104694535

%初始化
clc;
clear;
close all;

addpath('./func/');

%导入没有下采样的点云
src = pcread('./data/src.pcd').Location;
tgt = pcread('./data/tgt.pcd').Location;

% s = sum(src,1);
% center = (s-1)./size(s,1);
% src  = src -center;
% figure;
% pcshow(src,'green');hold on ;
% title('去质心的点云，其实就是做了一个平移');

figure;
pcshow(src,'green');hold on ;
pcshow(tgt,'red');
axis off;
title('icp配准前','FontSize',20);

%导入下采样的点云
srcd = downsampling(src,0.02);
tgtd = downsampling(tgt,0.02);


%原始点云与目标点云的大小不一定相同,以原始点数目为准
%closestpoints为最近点，我们假定他为目标点云
closestpoints = zeros(size(srcd,1),3);
distance = zeros(size(srcd,1),1);
helper_distance = zeros(size(srcd,1),3);
n = size(srcd,1);



iteration = 1;%迭代次数
max_iteration =30;%最大迭代次数

tic;

while(iteration <= max_iteration)
    clc;
    fprintf('当前迭代次数:%d\n',iteration);
    iteration = iteration+ 1;
    %对与原始点云的每个点，都要在目标点云找一个最近点
    for i =1:n
        [val,index] = findKnearestpoints(tgtd,srcd(i,:),1);
        closestpoints(i,:) = tgtd(index,:);
    end
    
    %找原始点云与目标点云的重心
    center_srcd = mean(srcd);
    center_closestpoints = mean(closestpoints);
    %去中心化的点云
    helper_srcd = srcd - center_srcd;
    helper_closestpoints = closestpoints - center_closestpoints;
    
    %没有疑惑了，直接看上面的专栏
    W = helper_srcd' * helper_closestpoints;
   
    %svd分解
    [u,s,v] = svd(W);
    %求r，t的最优解
    r = v * u';
    t = center_closestpoints' -   r * center_srcd';
  
    %对点云做刚体变换
    srcd = (r * srcd' + t)';
    src = (r * src'+ t)';
    
end

fprintf('icp算法\t');
toc;

figure;
pcshow(src,'green');hold on ;
pcshow(tgt,'red');
axis off;
title('icp配准后','FontSize',20);

