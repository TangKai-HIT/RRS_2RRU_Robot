function rotm = rotY(theta)
%ROTY 此处显示有关此函数的摘要
%   此处显示详细说明
rotm = [cos(theta)    0   sin(theta);
        0             1     0;
        -sin(theta)   0   cos(theta)];
end

