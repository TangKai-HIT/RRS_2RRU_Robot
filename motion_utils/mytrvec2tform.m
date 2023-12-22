function Tf = mytrvec2tform(vec)
%MYTRVEC2TFORM 此处显示有关此函数的摘要
%   此处显示详细说明
Tf = [1 0 0 vec(1);
          0 1 0 vec(2);
          0 0 1 vec(3);
          0 0 0  1];
end

