function rotm = rotX(theta)
%ROTX 此处显示有关此函数的摘要
%   此处显示详细说明
rotm = [1    0   0;
        0   cos(theta) -sin(theta);
        0   sin(theta) cos(theta)];
end

