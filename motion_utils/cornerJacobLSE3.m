function Q_L = cornerJacobLSE3(twist)
%CORNERJACOBLSE3 corner matrix (off-diagonal block matrix) of SE3 left jacobian 
theta = twist(1:3);
theta_hat = skewMatrix(theta);
theta_hat_sq = theta_hat*theta_hat;

p = twist(4:6);
p_hat = skewMatrix(p);

thetaNorm = norm(theta);
a = (thetaNorm - sin(thetaNorm)) / thetaNorm^3;
b = (thetaNorm^2 + 2*cos(thetaNorm) - 2) / (2*thetaNorm^4);
c = (2*thetaNorm - 3*sin(thetaNorm) + thetaNorm*cos(thetaNorm)) / (2*thetaNorm^5);

Q_L = 0.5*p_hat + a*(theta_hat*p_hat+p_hat*theta_hat+theta_hat*p_hat*theta_hat) + ...
                b*(theta_hat_sq*p_hat+p_hat*theta_hat_sq-3*theta_hat*p_hat*theta_hat) + ...
                c*(theta_hat*p_hat*theta_hat_sq + theta_hat_sq*p_hat*theta_hat);
end