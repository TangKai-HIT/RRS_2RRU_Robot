function [twist, theta] = logMapSE3(SE3)

twist = zeros(6,1);
R = SE3(1:3,1:3);
P = SE3(1:3, 4);

if trace((R-eye(3))'*(R-eye(3)))==0
    theta = norm(P);
    if theta>0
        twist(4:6) = P/theta;
    end

else
    [twist(1:3), theta] = logMapSO3(R);
    omegaSkew = skewMatrix(twist(1:3));
    G_inv = eye(3)/theta - 0.5*omegaSkew + (1/theta - 0.5*cot(theta/2)) * omegaSkew * omegaSkew;
    twist(4:6) = G_inv * P;
end

if nargout==1
    twist = twist * theta;
end