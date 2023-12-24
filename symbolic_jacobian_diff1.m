%symbolic differentiation of task traj-constrained jacobians of RRS-2RRU hybrid robot

%% Define vars
syms R r L1 L2 toolHight real positive
syms pkm_y_offset pkm_z_offset real
syms Pe [3,1] real
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

%% Trans to matlab functions
matlabFunction(configs,"File","Hyd_IK_1","Vars",{[alpha, beta], Pe, R, r, L1, L2, toolHight, pkm_y_offset pkm_z_offset}, "Optimize", true);

%% Get J_a2R, J_aPe (2R = [alpha; beta])
% Hyd_RRS_2RRU_Sym.getActuationBodyJacob();
% [J_a2R, J_a_Pe] = Hyd_RRS_2RRU_Sym.getEndPosConJacobian2R();

J_a2R(:, 1) = diff(configs, alpha);
J_a2R(:, 2) = diff(configs, beta);
J_a_Pe(:, 1) = diff(configs, Pe(1));
J_a_Pe(:, 2) = diff(configs, Pe(2));
J_a_Pe(:, 3) = diff(configs, Pe(3));

%% Trans to matlab functions
matlabFunction(J_a2R,"File","Hyd_Ja2R_1","Vars",{[alpha, beta], Pe, R, r, L1, L2, toolHight, pkm_y_offset pkm_z_offset}, "Optimize", true);
matlabFunction(J_a_Pe,"File","Hyd_JaPe_1","Vars",{[alpha, beta], Pe, R, r, L1, L2, toolHight, pkm_y_offset pkm_z_offset}, "Optimize", true);

%% Get dJ_a2R/d2R, dJ_aPe/d2R (2R = [alpha; beta])
dJ_a2R_d2R(:, :, 1) = diff(J_a2R, alpha);
dJ_a2R_d2R(:, :, 2) = diff(J_a2R, beta);

dJ_a2R_dPe(:, :, 1) = diff(J_a2R, Pe(1));
dJ_a2R_dPe(:, :, 2) = diff(J_a2R, Pe(2));
dJ_a2R_dPe(:, :, 3) = diff(J_a2R, Pe(3));

dJ_aPe_d2R(:, :, 1) = diff(J_a_Pe, alpha);
dJ_aPe_d2R(:, :, 2) = diff(J_a_Pe, beta);

dJ_aPe_dPe(:, :, 1) = diff(J_a_Pe, Pe(1));
dJ_aPe_dPe(:, :, 2) = diff(J_a_Pe, Pe(2));
dJ_aPe_dPe(:, :, 3) = diff(J_a_Pe, Pe(3));

%% Trans to matlab functions
matlabFunction(dJ_a2R_d2R,"File","Hyd_dJa2R_d2R_1","Vars",{[alpha, beta], Pe, R, r, L1, L2, toolHight, pkm_y_offset pkm_z_offset}, "Optimize", true);
matlabFunction(dJ_aPe_d2R,"File","Hyd_dJaPe_d2R_1","Vars",{[alpha, beta], Pe, R, r, L1, L2, toolHight, pkm_y_offset pkm_z_offset}, "Optimize", true);

matlabFunction(dJ_a2R_dPe,"File","Hyd_dJa2R_dPe_1","Vars",{[alpha, beta], Pe, R, r, L1, L2, toolHight, pkm_y_offset pkm_z_offset}, "Optimize", true);
matlabFunction(dJ_aPe_dPe,"File","Hyd_dJaPe_dPe_1","Vars",{[alpha, beta], Pe, R, r, L1, L2, toolHight, pkm_y_offset pkm_z_offset}, "Optimize", true);

%% Get ddJ_a2R/dd2R, , ddJ_aPe/dd2R (2R = [alpha; beta])
ddJ_a2R_dd2R(:, :, :, 1) = diff(dJ_a2R_d2R, alpha);
ddJ_a2R_dd2R(:, :, :, 2) = diff(dJ_a2R_d2R, beta);

ddJ_a2R_d2RdPe(:, :, :, 1) = diff(dJ_a2R_d2R, Pe(1));
ddJ_a2R_d2RdPe(:, :, :, 2) = diff(dJ_a2R_d2R, Pe(2));
ddJ_a2R_d2RdPe(:, :, :, 3) = diff(dJ_a2R_d2R, Pe(3));

ddJ_a2R_dPed2R(:, :, :, 1) = diff(dJ_a2R_dPe, alpha);
ddJ_a2R_dPed2R(:, :, :, 2) = diff(dJ_a2R_dPe, beta);

ddJ_a2R_ddPe(:, :, :, 1) = diff(dJ_a2R_dPe, Pe(1));
ddJ_a2R_ddPe(:, :, :, 2) = diff(dJ_a2R_dPe, Pe(2));
ddJ_a2R_ddPe(:, :, :, 3) = diff(dJ_a2R_dPe, Pe(3));

ddJ_aPe_dd2R(:, :, :, 1) = diff(dJ_aPe_d2R, alpha);
ddJ_aPe_dd2R(:, :, :, 2) = diff(dJ_aPe_d2R, beta);

ddJ_aPe_d2RdPe(:, :, :, 1) = diff(dJ_aPe_d2R, Pe(1));
ddJ_aPe_d2RdPe(:, :, :, 2) = diff(dJ_aPe_d2R, Pe(2));
ddJ_aPe_d2RdPe(:, :, :, 3) = diff(dJ_aPe_d2R, Pe(3));

ddJ_aPe_dPed2R(:, :, :, 1) = diff(dJ_aPe_dPe, alpha);
ddJ_aPe_dPed2R(:, :, :, 2) = diff(dJ_aPe_dPe, beta);

ddJ_aPe_ddPe(:, :, :, 1) = diff(dJ_aPe_dPe, Pe(1));
ddJ_aPe_ddPe(:, :, :, 2) = diff(dJ_aPe_dPe, Pe(2));
ddJ_aPe_ddPe(:, :, :, 3) = diff(dJ_aPe_dPe, Pe(3));

%% Trans to matlab functions
matlabFunction(ddJ_a2R_dd2R,"File","Hyd_ddJa2R_dd2R_1","Vars",{[alpha, beta], Pe, R, r, L1, L2, toolHight, pkm_y_offset pkm_z_offset}, "Optimize", true);
matlabFunction(ddJ_a2R_d2RdPe,"File","Hyd_ddJa2R_d2RdPe_1","Vars",{[alpha, beta], Pe, R, r, L1, L2, toolHight, pkm_y_offset pkm_z_offset}, "Optimize", true);
matlabFunction(ddJ_a2R_dPed2R,"File","Hyd_ddJa2R_dPed2R_1","Vars",{[alpha, beta], Pe, R, r, L1, L2, toolHight, pkm_y_offset pkm_z_offset}, "Optimize", true);
matlabFunction(ddJ_a2R_ddPe,"File","Hyd_ddJa2R_ddPe_1","Vars",{[alpha, beta], Pe, R, r, L1, L2, toolHight, pkm_y_offset pkm_z_offset}, "Optimize", true);

matlabFunction(ddJ_aPe_dd2R,"File","Hyd_ddJaPe_dd2R_1","Vars",{[alpha, beta], Pe, R, r, L1, L2, toolHight, pkm_y_offset pkm_z_offset}, "Optimize", true);
matlabFunction(ddJ_aPe_d2RdPe,"File","Hyd_ddJaPe_d2RdPe_1","Vars",{[alpha, beta], Pe, R, r, L1, L2, toolHight, pkm_y_offset pkm_z_offset}, "Optimize", true);
matlabFunction(ddJ_aPe_dPed2R,"File","Hyd_ddJaPe_dPed2R_1","Vars",{[alpha, beta], Pe, R, r, L1, L2, toolHight, pkm_y_offset pkm_z_offset}, "Optimize", true);
matlabFunction(ddJ_aPe_ddPe,"File","Hyd_ddJaPe_ddPe_1","Vars",{[alpha, beta], Pe, R, r, L1, L2, toolHight, pkm_y_offset pkm_z_offset}, "Optimize", true);
