%symbolic differentiation of task traj-constrained jacobians of RRS-2RRU hybrid robot

%% Define vars
syms R r L1 L2 toolHight pkm_y_offset pkm_z_offset real
syms Pe [3,1] real
syms dPe [3,1] real
syms ddPe [3,1] real
syms alpha beta real

%% Init symbolic obj
% R = 104*1e-3;
% r = 104*1e-3;
% L1 = 208*1e-3;
% L2 = 249.6*1e-3; 
% toolHight = 136.8*1e-3;
% pkm_y_offset = 0*1e-3;
% pkm_z_offset = 21.6*1e-3;
z_p_min = 201.937*1e-3;
d1_init = 490*1e-3;

RRS_2RRU_Sym = RRS_2RRU_Basic_Syms(R, r, L1, L2, toolHight, z_p_min);
Hyd_RRS_2RRU_Sym = RRS_2RRU_Hybrid_Syms(RRS_2RRU_Sym, z_p_min, d1_init, pkm_y_offset, pkm_z_offset);

%% Get constrained IK
configs = Hyd_RRS_2RRU_Sym.getEndPosConInvKine2R(Pe, alpha, beta);

%% Get J_a2R, J_aPe (2R = [alpha; beta])
Hyd_RRS_2RRU_Sym.getActuationBodyJacob();
[J_a2R, J_a_Pe] = Hyd_RRS_2RRU_Sym.getEndPosConJacobian2R();

%% Get dJ_a2R/d2R, dJ_aPe/d2R (2R = [alpha; beta])
dJ_a2R_d2R(:, :, 1) = diff(J_a2R, alpha);
dJ_a2R_d2R(:, :, 2) = diff(J_a2R, beta);

dJ_aPe_d2R(:, :, 1) = diff(J_a_Pe, alpha);
dJ_aPe_d2R(:, :, 2) = diff(J_a_Pe, beta);

%% Trans to matlab functions
matlabFunction(dJ_a2R_d2R,"File","Hyd_dJa2R_d2R","Vars",{[alpha, beta], Pe, R, r, L1, L2, toolHight, pkm_y_offset pkm_z_offset}, "Optimize", true);
matlabFunction(dJ_aPe_d2R,"File","Hyd_dJaPe_d2R","Vars",{[alpha, beta], Pe, R, r, L1, L2, toolHight, pkm_y_offset pkm_z_offset}, "Optimize", true);

%% Get ddJ_a2R/dd2R, , ddJ_aPe/dd2R (2R = [alpha; beta])
ddJ_a2R_dd2R(:, :, :, 1) = diff(dJ_a2R_d2R, alpha);
ddJ_a2R_dd2R(:, :, :, 2) = diff(dJ_a2R_d2R, beta);

ddJ_aPe_dd2R(:, :, :, 1) = diff(dJ_aPe_d2R, alpha);
ddJ_aPe_dd2R(:, :, :, 2) = diff(dJ_aPe_d2R, beta);

%% Trans to matlab functions
matlabFunction(ddJ_a2R_dd2R,"File","Hyd_ddJa2R_dd2R","Vars",{[alpha, beta], Pe, R, r, L1, L2, toolHight, pkm_y_offset pkm_z_offset}, "Optimize", true);
matlabFunction(ddJ_aPe_dd2R,"File","Hyd_ddJaPe_dd2R","Vars",{[alpha, beta], Pe, R, r, L1, L2, toolHight, pkm_y_offset pkm_z_offset}, "Optimize", true);
