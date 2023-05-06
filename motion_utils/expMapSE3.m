function SE3 = expMapSE3(twist, theta)

omega = twist(1:3);
v = twist(4:6);
if size(v,2)~=1
    v = v';
end

omegaNorm = norm(omega);
if omegaNorm>0
        omega = omega/omegaNorm;
end

if ~exist("theta","var")
    theta = omegaNorm;
else
    v = v * theta;
    theta = theta * omegaNorm;
end

if omegaNorm>0
    omegaSkew = skewMatrix(omega);
    SO3 = eye(3) + omegaSkew * sin(theta) + omegaSkew*omegaSkew*(1-cos(theta));
    
    SE3 = [SO3,   (eye(3)*theta + (1-cos(theta))*omegaSkew + (theta - sin(theta))*omegaSkew*omegaSkew) * v;
                zeros(1,3),                                 1];
else
    SE3 = [eye(3),           v;
                zeros(1,3),     1];
end
