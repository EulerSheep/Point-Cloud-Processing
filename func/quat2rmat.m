function [R] = quat2rmat(w,x,y,z)

%把四元数转换为旋转矩阵r

R = zeros(3,3);
R(1,1) = 1-2*y*y-2*z*z;
R(1,2) = 2*x*y+2*w*z;
R(1,3) = 2*x*z-2*w*y;
R(2,1) = 2*x*y-2*w*z;
R(2,2) = 1-2*x*x-2*z*z;
R(2,3) = 2*y*z+2*w*x;
R(3,1)= 2*x*z+2*w*y;
R(3,2) = 2*y*z-2*w*x;
R(3,3) = 1-2*x*x-2*y*y;

end

