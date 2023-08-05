function theta = subIK_rotateOneAxis(twist,startPt, endPt)
%SUBIK_ROTATEONEAXIS Paden Kahan inverse kinematic subproblem of rotate about one axis
%   Inputs:
%       twist:6 X 1, [omega; rXomega] zero pitch unit twist
%       startPt: 
%       endPt

omega = twist(1:3);
v = twist(4:6);

q = cross(omega, v);

u = startPt - q;
v = endPt - q;

proj = omega * omega';
u_n = u - proj*u;
v_n = v - proj*v;

if norm(u_n) > 1e-6
    theta = atan2(omega'*cross(u_n, v_n), u_n'*v_n);
else
    theta = [];
end

end
