function [featurevalue,featurevector] = calfeaturevv(A,flag)
%找最大的特征值与特征向量
%求矩阵A的全部特征值，构成对角阵y;特征向量构成矩阵x
[x,y] = eig(A);%求特征值，特征向量
lambda = diag(y);%找到对角线的元素

if(flag == 'max')
    [featurevalue,i] = max(lambda);
    featurevector = x(:,i);
elseif(flag == 'min')
    [featurevalue,i] = min(lambda);
    featurevector = x(:,i);
end

end

