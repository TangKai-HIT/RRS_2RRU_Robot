function outputKines = RRS_2RRU_fkine(thetas, R, r, L1, L2, z_p0, alpha0, beta0)
%RRS_2RRU_FKINE solve forward kinematics of RRS_2RRU robot
%   Inputs:
%       thetas: 1 X 3 or 3 X 1 
%       z_p0, alpha0, beta0: init guess of output motion params
%   Outputs:
%       outputKines: [z_p, alpha, beta]

fun = @(x) closeLoopFuncs(x, thetas, R, r, L1, L2);
x0 = [z_p0, alpha0, beta0]';

options = optimoptions('fsolve','Display','final','Algorithm','levenberg-marquardt');
outputKines = fsolve(fun, x0, options);

% outputKines = fsolve(fun, x0);

end

%% close-loop vector constraints
function fun = closeLoopFuncs(x, thetas, R, r, L1, L2)
% x=[z_p; alpha; beta]
    cur_z_p = x(1);
    cur_alpha = x(2);
    cur_beta = x(3);

   p = [-sin(cur_alpha)*sin(cur_beta)*r; 0; cur_z_p];
   R_p = [cos(cur_beta),  sin(cur_alpha)*sin(cur_beta),  cos(cur_alpha)*sin(cur_beta);
                    0   ,       cos(cur_alpha),         -sin(cur_alpha);
                -sin(cur_beta), sin(cur_alpha)*cos(cur_beta),  cos(cur_alpha)*cos(cur_beta)]; 
   e_x = [1;0;0];  
   e_y = [0;1;0];
    
   theta11 = thetas(1);
   theta21 = thetas(2);
   theta31 = thetas(3);
   
   p_A1 = [R; 0; 0];
   p_A2 = [-R; 0; 0];
   p_A3 = [0; R; 0]; 

   fun(1) = norm((R_p*R*e_x + p) - (L1*rotY(-theta11)*e_x+p_A1)) - L2;
   fun(2) = norm((-R_p*R*e_x + p) - (L1*rotY(-theta21)*e_x+p_A2)) - L2;
   fun(3) = norm((R_p*R*e_y + p) - (L1*rotX(theta31)*e_y+p_A3)) - L2;
end