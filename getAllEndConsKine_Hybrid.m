function [state_config, J_a_2R, J_a_Pe, dJ_a_2R, dJ_a_Pe] = getAllEndConsKine_Hybrid(state_2R, state_Pe, R, r, L1, L2, toolHight, pkm_y_offset, pkm_z_offset)
%GETALLENDCONSKINE_HYBRID get all end-position(tool tip position in base frame) constrained inverse kinematics states w.r.t 2R
%   Inputs:
%       state_2R: 6 X 1, [alpha;beta;dalpha;dbeta;ddalpha;ddbeta]  
%       state_Pe: 9 X 1, [Pe;dPe;ddPe]
%   Outputs:
%       state_config: 15 X 1, [config; dconfig; ddconfig]
%       J_a2R, J_a_Pe: 

state_config = zeros(15, 1);

P_tool = state_Pe(1:3);
dP_tool = state_Pe(4:6);

alpha = state_2R(1);
beta = state_2R(2);

twist_1 = [1; 0; 0; 0; 0; 0]; %twist of first serial: rotational joint
twist_2 = [0; 0; 0; 1; 0; 0]; %twist of second serial: prismatic joint

A1 = [R; 0; 0];
A2 = [-R; 0; 0];
A3 = [0; R; 0];

C1_p = [r; 0; 0];
C2_p = [-r; 0; 0];
C3_p = [0; r; 0];

s11 = [0;-1;0]; %theta11
s12 = [0;-1;0]; %theta12
s21 = [0;-1;0]; %theta21
s22 = [0;-1;0]; %theta22
s31 = [1;0;0]; %theta31
s32 = [1;0;0]; %theta32

%% IK part
%get d1
l_pkm_tool = rotY(beta)*rotX(alpha)*[0; 0; toolHight];
delta_Px = -sin(alpha)*sin(beta)*r;
d1 = P_tool(1) - l_pkm_tool(1) - delta_Px;

%get z_p
z_p = sqrt(P_tool(2:3)'*P_tool(2:3) - (l_pkm_tool(2)+pkm_y_offset)^2) - (pkm_z_offset+l_pkm_tool(3));

%get thetas in pkm
R_pkm_P = [cos(beta),  sin(alpha)*sin(beta),  cos(alpha)*sin(beta);
        0   ,       cos(alpha),         -sin(alpha);
        -sin(beta), sin(alpha)*cos(beta),  cos(alpha)*cos(beta)];

P_pkm_0 = [-sin(alpha)*sin(beta)*r; 0; z_p];

Tf_pkm_P = [R_pkm_P, P_pkm_0;
        zeros(1,3),  1];

btc_p = [0, 0, toolHight];
Tf_P_tool = trvec2tform(btc_p);
Ad_P_tool = adjointMatrix(Tf_P_tool);
% Tf_BTC_local = Tf_pkm_P * Tf_P_tool;    

C1_hom = Tf_pkm_P * [C1_p; 1];
C2_hom = Tf_pkm_P * [C2_p; 1];
C3_hom = Tf_pkm_P * [C3_p; 1];

C1 = C1_hom(1:3);
C2 = C2_hom(1:3);
C3 = C3_hom(1:3);

vec_A1C1 = C1 - A1;
vec_A2C2 = C2 - A2;
vec_A3C3 = C3 - A3;

thetas_pkm = zeros(3,1);
%theta11(R-R-R crank)
delta = atan2(vec_A1C1(3), vec_A1C1(1));
d = norm(vec_A1C1);
gamma = acos((d^2 + L1^2 - L2^2)/(2*d*L1));
thetas_pkm(1) = delta - gamma;

%theta12(R-R-R crank)
delta = atan2(vec_A2C2(3), vec_A2C2(1));
d = norm(vec_A2C2);
gamma = acos((d^2 + L1^2 - L2^2)/(2*d*L1));
thetas_pkm(2) = gamma + delta;

%theta13(R-R-R crank)
delta = atan2(vec_A3C3(3), vec_A3C3(2));
d = norm(vec_A3C3);
gamma = acos((d^2 + L1^2 - L2^2)/(2*d*L1));
thetas_pkm(3) = delta - gamma;

%update Bi
B1 = A1 + rotY(-thetas_pkm(1)) * [L1; 0; 0];
B2 = A2 + rotY(-thetas_pkm(2)) * [L1; 0; 0];
B3 = A3 + rotX(thetas_pkm(3)) * [0; L1; 0];

%update dirctional vectors
b1 = (B1 - A1)/L1;
b2 = (B2 - A2)/L1;
b3 = (B3 - A3)/L1;

c11 = (C1 - B1)/L2;
c21 = (C2 - B2)/L2;
c31 = (C3 - B3)/L2;

c12 = (C1 - P_pkm_0)/r;
c22 = (C2 - P_pkm_0)/r;
c32 = (C3 - P_pkm_0)/r;

%get theta1
P_1 = [d1+delta_Px; pkm_y_offset; z_p+pkm_z_offset];
theta1 = subIK_rotateOneAxis(twist_1, P_1+l_pkm_tool, P_tool);

%upate Tf_BTC
ExpSO3_theta1 = expMapSO3(twist_1(1:3), theta1);
rotm_P = ExpSO3_theta1 * rotY(beta) * rotX(alpha);
P = ExpSO3_theta1 * P_1;
% Tf_BTC = [rotm_P, P_tool;
%                           zeros(1,3), 1];
Tf_P = [rotm_P,   P;
                      zeros(1,3), 1];

%update Tf_0_P0, Tf_0_pkm, Tf_pkm_P
Tf_0_P0 = expMapSE3(twist_1, theta1) * expMapSE3(twist_2, d1);
% Tf_0_pkm = Tf_0_P0 * trvec2tform([0, pkm_y_offset, pkm_z_offset]);

Tf_C3_P = trvec2tform([0,-r, 0]);
Ad_C3_P = adjointMatrix(Tf_C3_P);

state_config(1:5) = [theta1; d1; thetas_pkm];

%% PKM actuation jacobian part
J_theta_a = L1 * blkdiag(dot(cross(b1,c11), s11), dot(cross(b2,c21), s21), dot(cross(b3,c31), s31));
            
J_X_1a = [r*(cross(c12,c11)'), c11'];
J_X_2a = [r*(cross(c22,c21)'), c21'];
J_X_3a = [r*(cross(c32,c31)'), c31'];

J_X_a = [J_X_1a; J_X_2a; J_X_3a];

J_a_pkm = J_theta_a \ J_X_a;

%% PKM passive jacobian part
J_pa_1 = [r*cross(c12, b1)', b1']./(L2*dot(cross(c11, b1), s12));
J_pa_2 = [r*cross(c22, b2)', b2']./(L2*dot(cross(c21, b2), s22));
J_pa_3 = [r*cross(c32, b3)', b3']./(L2*dot(cross(c31, b3), s32));
J_pa_pkm = [J_pa_1; J_pa_2; J_pa_3];

%% Hybrid robot actuation jacobian part
I_tilt = [0, -1; 1, 0];

Tf_P_0 = getInvSE3(Tf_P);
Tf_P_P0 = Tf_P_0 * Tf_0_P0;

p_yz = Tf_P(2:3, 4);
r_sq = p_yz' * p_yz;

rotm_pkm_P = Tf_pkm_P(1:3, 1:3);
blk_rotm_pkm_P = blkdiag(rotm_pkm_P, rotm_pkm_P);

e4 = [0, 0, 0, 1, 0, 0];
e5 = [0, 0, 0, 0, 1, 0];
e6 = [0, 0, 0, 0, 0, 1];

%P-Point body twist actuation jacobian(in body frame)
J_a_P_1 = - 1/r_sq * p_yz' * I_tilt * [e5; e6] * blkdiag(rotm_P, rotm_P);

J_a_P_2 = e4 * blkdiag(rotm_P, rotm_P) * Ad_C3_P;

J_Opkm_P = adjointMatrix(Tf_P_P0) * (adjointMatrix(expMapSE3(twist_2, -d1)) * twist_1 * J_a_P_1 + twist_2 * J_a_P_2);
J_pkmP_P = eye(6) - J_Opkm_P;
J_a_P_3 = J_a_pkm *  blk_rotm_pkm_P * J_pkmP_P;

J_a_P = [J_a_P_1; J_a_P_2; J_a_P_3];

%tool tip body twist actuation jacobian(in body frame)
J_a_tool = J_a_P * Ad_P_tool;

%% End position constrained actuation jacobian part
rot_alpha_inv = rotX(-alpha);
rot_beta_inv = rotY(-beta);
s_theta1 = rot_alpha_inv*rot_beta_inv*[1;0;0];
s_beta = rot_alpha_inv*[0;1;0];
s_alpha = [1;0;0];
J_2R_omega = [s_alpha, s_beta];

k_2R = 1 - J_a_tool(1, 1:3)*s_theta1;
J_2R_theta1 = J_a_tool(1, 1:3)/k_2R * J_2R_omega;
J_Pe_theta1 = J_a_tool(1, 4:6)/k_2R;

J_a_2R = J_a_tool(:, 1:3)*(s_theta1*J_2R_theta1 + J_2R_omega);
J_a_Pe_b = J_a_tool(:, 1:3)*s_theta1*J_Pe_theta1 + J_a_tool(:, 4:6);
J_a_Pe = J_a_Pe_b * Tf_P(1:3, 1:3)';

state_config(6:10) = J_a_2R*state_2R(3:4) + J_a_Pe*state_Pe(4:6);

%% End position constrained actuation jacobian diff part
d2R = state_2R(3:4);

J_a_Pe_b = J_a_tool(:, 1:3)*s_theta1*J_Pe_theta1 + J_a_tool(:, 4:6);

dP_tool_b = rotm_P' * dP_tool;
dtheta_1 = J_2R_theta1*d2R + J_Pe_theta1*dP_tool_b;

omega_tool_b = s_theta1*dtheta_1 + s_beta*d2R(2) + s_alpha*d2R(1);
dX_tool_b = [omega_tool_b; dP_tool_b];
%%Hybrid robot jacobian diff
dX_P = Ad_P_tool * dX_tool_b;

%dJ_a for theta1
I_tilt = [0, -1; 1, 0];

p_yz = Tf_P(2:3, 4);
dp = rotm_P * dX_P(4:6);
dp_yz = dp(2:3);
r_sq = p_yz' * p_yz;

omega_P_vedge = skewMatrix(dX_P(1:3));
temp = I_tilt * [e5; e6] * blkdiag(rotm_P, rotm_P);
dJ_a_P1 = -1/r_sq * (dp_yz' * temp + p_yz' * temp * blkdiag(omega_P_vedge, omega_P_vedge) + 2*dp_yz'*p_yz*J_a_P(1,:));

%dJ_a for d1
dJ_a_P2 = e4 * blkdiag(rotm_P, rotm_P) * blkdiag(omega_P_vedge, omega_P_vedge) * Ad_C3_P;

%dJ_a for thetas in pkm
dX_pkm_P = J_pkmP_P * dX_P;
ad_dX_P0_P = adjointTwist(dX_pkm_P);

Ad_P_P0 = adjointMatrix(getInvSE3(Tf_P) * Tf_0_P0);

d_d1 = J_a_P(2, :) * dX_P;
ad_dX_Opkm = adjointTwist(twist_2*d_d1);
Ad_dX_p0O = adjointMatrix(expMapSE3(twist_2, -d1));
dJ_pkmP_P = ad_dX_P0_P*J_Opkm_P - Ad_P_P0*(-ad_dX_Opkm*Ad_dX_p0O*twist_1*J_a_P(1, :) + ...
                            Ad_dX_p0O*twist_1*dJ_a_P1 + twist_2*dJ_a_P2);

dX_p_pkm = blk_rotm_pkm_P*dX_pkm_P;

%%PKM jacobian diff
E = [eye(3), zeros(3,3)];
dtheta1 = J_a_pkm*dX_p_pkm;
dtheta2 = J_pa_pkm*dX_p_pkm;
omega_p = dX_p_pkm(1:3);

nom = L1*dot(cross(b1,c11), s11);
dJ_a1 = (r*((c11'*omega_p)*c12' - (c11'*c12)*omega_p')*E +...
            L1*c11'*b1*dtheta1(1)*J_a_pkm(1,:) + L2*dtheta2(1)*J_pa_pkm(1,:))./nom;

nom = L1*dot(cross(b2,c21), s21);
dJ_a2 = (r*((c21'*omega_p)*c22' - (c21'*c22)*omega_p')*E +...
            L1*c21'*b2*dtheta1(2)*J_a_pkm(2,:) + L2*dtheta2(2)*J_pa_pkm(2,:))./nom;

nom = L1*dot(cross(b3,c31), s31);
dJ_a3 = (r*((c31'*omega_p)*c32' - (c31'*c32)*omega_p')*E +...
            L1*c31'*b3*dtheta1(3)*J_a_pkm(3,:) + L2*dtheta2(3)*J_pa_pkm(3,:))./nom;                      

dJ_a_pkm = [dJ_a1; dJ_a2; dJ_a3];

%jacobian diff for thetas
omega_pmk_P_vedge = skewMatrix(dX_pkm_P(1:3));
dJ_a_P35 = dJ_a_pkm*blk_rotm_pkm_P*J_pkmP_P + J_a_pkm*blk_rotm_pkm_P*(blkdiag(omega_pmk_P_vedge, omega_pmk_P_vedge)*J_pkmP_P + dJ_pkmP_P);

%jacobian diff for all
dJ_a_P = [dJ_a_P1; dJ_a_P2; dJ_a_P35];
dJ_a_tool = dJ_a_P * Ad_P_tool;

ex = [1;0;0];
ey=[0;1;0];
ds_theta1_d2R = -[rot_alpha_inv*skewMatrix(ex)*rot_beta_inv*ex, rot_alpha_inv*rot_beta_inv*skewMatrix(ey)*ex];
ds_theta1 = ds_theta1_d2R*d2R;
ds_beta = rot_alpha_inv*skewMatrix(ex)*ey*(-d2R(1));
dJ_2R_omega = [zeros(3,1), ds_beta];
dk_2R = -(dJ_a_tool(1, 1:3)*s_theta1 + J_a_tool(1, 1:3)*ds_theta1);
dJ_2R_theta1 = (dJ_a_tool(1, 1:3)*J_2R_omega + J_a_tool(1, 1:3)*dJ_2R_omega - dk_2R*J_2R_theta1) / k_2R;
dJ_Pe_theta1 = (dJ_a_tool(1, 4:6) - dk_2R*J_Pe_theta1) / k_2R;

dJ_a_2R = dJ_a_tool(:, 1:3)*(s_theta1*J_2R_theta1 + J_2R_omega) + ...
                    J_a_tool(:, 1:3)*(ds_theta1*J_2R_theta1 + s_theta1*dJ_2R_theta1 + dJ_2R_omega);
dJ_a_Pe_b = dJ_a_tool(:, 1:3)*s_theta1*J_Pe_theta1 + J_a_tool(:, 1:3)*ds_theta1*J_Pe_theta1+ ...
                    J_a_tool(:, 1:3)*s_theta1*dJ_Pe_theta1 + dJ_a_tool(:, 4:6);
dJ_a_Pe = (dJ_a_Pe_b + J_a_Pe_b*skewMatrix(-omega_tool_b)) * Tf_P(1:3, 1:3)';

state_config(11:15) = J_a_2R*state_2R(5:6) + J_a_Pe*state_Pe(7:9) + dJ_a_2R*state_2R(3:4) + dJ_a_Pe*state_Pe(4:6);

end

