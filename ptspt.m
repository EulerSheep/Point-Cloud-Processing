% 四元数配准算法实现
% 当目标点云与原始点云点一一对应，可以直接找到刚体变换
% 代码参考 https://blog.csdn.net/qq_36686437/article/details/108587997
clc;
clear;
close all;

%导入没有下采样的点云
src = pcread('./data/src.pcd').Location;
%导入下采样的点云
srcd = pcread('./data/src_down.pcd').Location;

%设置刚体变换矩阵
real_r = [0.104639960054491,-0.897108883255251,0.429239013073499;
    -0.0881158057378587,0.421547684869394,0.902514905228951;
    -0.990598850967920,-0.132261865122835,-0.0349387391768306];
real_t = [-0.52113783;0.048614666;-1.0154600];

tgtd = srcd *real_r  + real_t';
tgt =  src *real_r  + real_t';

figure;
pcshow(srcd,'green');hold on ;
pcshow(tgtd,'red');
title('ptspt配准前','FontSize',20);

closestpoints = zeros(size(srcd,1),3);
iteration = 1;
n = size(srcd,1);
max_iteration = 1;

while(iteration <= max_iteration)

    fprintf('当前迭代次数:%d\n',iteration);
    iteration = iteration+ 1;
    closestpoints = tgtd;
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
    r = quat2dcm([q0 q1 q2 q3]);
    t = center_closestpoints - center_srcd*r;
    srcd = srcd * r + t;
    src = src*r + t;
    
end

%看一下误差
error_r = mean(r - real_r)
error_t = mean(t - real_t)

figure;
pcshow(src,'green');hold on ;
pcshow(tgt,'red');
axis off;
title('ptspt配准后','FontSize',20);