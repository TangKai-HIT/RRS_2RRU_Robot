function adj_V = adjointTwist(twist)
%ADJOINTTWIST adjoint matrix of twist, for Lie bracket computation
%   此处显示详细说明
omega_vedge = skewMatrix(twist(1:3));
adj_V = [omega_vedge, zeros(3,3);
                skewMatrix(twist(4:6)), omega_vedge];
end

