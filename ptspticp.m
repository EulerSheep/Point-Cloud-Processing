% 修改了一下icp算法，看能不能用ptspt实现
% 这个不用 源点云与目标点云一一对应
% 就是把icp算法求解最优r,t的方法，换成了ptspt的算法
% 结论是效果还行
% 代码参考 https://blog.csdn.net/qq_36686437/article/details/108587997

clc;
clear;
close all;

addpath('./func/');

%导入下采样的点云
srcd = pcread('./data/src_down.pcd').Location;
tgtd = pcread('./data/tgt_down.pcd').Location;

figure;
pcshow(srcd,'green');hold on ;
pcshow(tgtd,'red');
title('修改后的icp-配准前','FontSize',20);

closestpoints = zeros(size(srcd,1),3);
iteration = 1;
n = size(srcd,1);
max_iteration = 30;

tic;
while(iteration <= max_iteration)

    fprintf('当前迭代次数:%d\n',iteration);
    iteration = iteration+ 1;
    
    for i =1:n
        [~,index] = findKnearestpoints(tgtd,srcd(i,:),1);
        %找到目标点云的最近点了，赋值
        closestpoints(i,:) = tgtd(index,:);
    end
    
    %找原始点云与目标点云的重心
    center_srcd = mean(srcd);
    center_closestpoints = mean(closestpoints);
    %去中心化的点云
    helper_srcd = srcd - center_srcd;
    helper_closestpoints = closestpoints - center_closestpoints;
    %构造协方差矩阵
    RC = helper_srcd' * helper_closestpoints/n;
    helperRC = RC - RC';
    %构造4*4矩阵
    RQ = zeros(4,4);
    RQ(1,1) = trace(RC);
    deltamat = [helperRC(2,3),helperRC(3,1),helperRC(1,2)]';
    RQ(1,2:4) = deltamat;
    RQ(2:4,1) = deltamat';
    RQ(2:4,2:4) = RC + RC' - trace(RC)*eye(3,3);
    %找到RQ最大特征值对应的特征向量
    %计算特征值与特征向量
    [x,y] = eig(RQ);%求矩阵Rq的全部特征值，构成对角阵y;特征向量构成矩阵x
    e = diag(y);%对角阵
    %======计算最大特征值对应的特征向量======
    lamda=max(e);%最大特征值
    for i=1:length(RQ)
        if lamda==e(i)%寻找最大特征值的位置
            break;
        end
    end
    q=x(:,i);%i表示最大特征向量的位置
    q0=q(1);q1=q(2);q2=q(3);q3=q(4);
%     r = quat2dcm([q0 q1 q2 q3]);%matlab自带
    r = quat2rmat(q0 ,q1, q2 ,q3);%自己写的
    t = center_closestpoints - center_srcd*r;
    srcd = srcd * r + t;
  
end
fprintf('修改后的icp算法\t');
toc;
%显示配准后的点云
figure;
pcshow(srcd,'green');hold on ;
pcshow(tgtd,'red');
axis off;
title('ptspt配准后---点云分布','FontSize',20);