function [val,index] = findKnearestpoints(tgtd,point,K)
%在tgtd中找到与point最近的K个点
%没有用matlab自带的kdtree，那个函数的输入太麻烦了，不方便后续处理

%求当前查询点，与目标点云每一点的x,y,z坐标差
helper_distance = tgtd - point;
%求x,y,z的平方
helper_distance = helper_distance.^2;
%按每一行x,y,z坐标加起来相加 得到当前查询点到目标点云每一点的距离的平方 dim = 2
distance = sum(helper_distance,2);
%找到最近点的索引index
% [~,index] = min(distance);
%找到目标点云的最近点了，赋值

[val,index] = sort(distance);
index = index(1:K);
val = val(1:K);

end

