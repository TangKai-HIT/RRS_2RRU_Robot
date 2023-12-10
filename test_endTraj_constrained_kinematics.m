%test 2RRU_RRS_Hybrid constrained tool tip trajecory kinematics 
clc; clear; close all;

%% Init params
R = 104*1e-3;
r = 104*1e-3;
L1 = 208*1e-3;
L2 = 249.6*1e-3; 
toolHight = 136.8*1e-3;
z_p_min = 201.937*1e-3;
pkm_y_offset = 0*1e-3;
pkm_z_offset = 21.6*1e-3;
d1_init = 490*1e-3;

RRS_2RRU = RRS_2RRU_Basic(R, r, L1, L2, toolHight, z_p_min);
Hyd_RRS_2RRU = RRS_2RRU_Hybrid(RRS_2RRU, z_p_min, d1_init, pkm_y_offset, pkm_z_offset);

%% Init end traj & expected states
wayPoints = [-pi/6, d1_init, 250*1e-3, 0, 0;
            pi/6, d1_init + 0.3, 350*1e-3, pi/8, -pi/8]';
timePoints = [0, 3];
N = 2000;
dt = (timePoints(end) - timePoints(1)) / (N-1);
time_sequence = linspace(timePoints(1), timePoints(end), N);
[q,qd,qdd,~] = cubicpolytraj(wayPoints,timePoints,time_sequence);

alpha_traj = q(4, :);
dalpha_traj = qd(4, :);
ddalpha_traj = qdd(4, :);

beta_traj = q(5, :);
dbeta_traj = qd(5, :);
ddbeta_traj = qdd(5, :);

Tf_tool_path = zeros(4,4,N);
endPos_traj = zeros(3, N);
configs_expect = zeros(5, N);
for i=1:N
    Tf_tool_path(:, :, i) = Hyd_RRS_2RRU.forwardKine(q(1, i), q(2,i), q(3,i), q(4,i), q(5,i));
    endPos_traj(:,i) = Tf_tool_path(1:3,4,i);

    configs_expect(:, i) = Hyd_RRS_2RRU.invKine(Tf_tool_path(:, :, i));
end

Diff1_Mat = getDiffMatrix_quadInterp(time_sequence, 1);
Diff2_Mat = getDiffMatrix_quadInterp(time_sequence, 2);

endVel_traj = (Diff1_Mat*endPos_traj')';
endAcc_traj = (Diff2_Mat*endPos_traj')';

dconfigs_expect = (Diff1_Mat*configs_expect')';
ddconfigs_expect = (Diff2_Mat*configs_expect')';

%% test IK , jacobian, jacobian derivative by given alpha, beta, P
configs_test = zeros(5, N);
dconfigs_test = zeros(5, N);
ddconfigs_test = zeros(5, N);

alpha_test = zeros(1, N);
beta_test = zeros(1, N);

Tf_tool_test = zeros(4, 4, N);
Tf_P_test = zeros(4, 4, N);

J_a_2R = zeros(5, 2, N);
J_a_Pe = zeros(5, 3, N);
dJ_a_2R = zeros(5, 2, N);
dJ_a_Pe = zeros(5, 3, N);

ddtheta1_test = zeros(1, N);
dX_tool_b_test = zeros(6, N);
dX_tool_b_expect = zeros(6, N-1);
dP_tool_test = zeros(3, N-1);
J_a_tool = zeros(5, 6, N);

for i=1:N
    configs_test(:, i) = Hyd_RRS_2RRU.getEndPosConInvKine2R(endPos_traj(:, i), alpha_traj(i), beta_traj(i));
    Tf_P_test(:, :, i) = Hyd_RRS_2RRU.Tf_P;
    Tf_tool_test(:, :, i) = Hyd_RRS_2RRU.Tf_BTC;
    alpha_test(i) = Hyd_RRS_2RRU.RRS_2RRU.alpha;
    beta_test(i) = Hyd_RRS_2RRU.RRS_2RRU.beta;

    [J_a_tool(:, :, i), ~, ~] = Hyd_RRS_2RRU.getActuationBodyJacob();

    [J_a_2R(:, :, i), J_a_Pe(:, :, i)] = Hyd_RRS_2RRU.getEndPosConJacobian2R();
    dconfigs_test(:, i) = J_a_2R(:, :, i)*[dalpha_traj(i); dbeta_traj(i)] + J_a_Pe(:, :, i)*endVel_traj(:, i);

    [dJ_a_2R(:, :, i), dJ_a_Pe(:, :, i)] = Hyd_RRS_2RRU.getEndPosConJacobian2RDiff(dalpha_traj(i), dbeta_traj(i), endVel_traj(:, i));
    ddconfigs_test(:, i) = J_a_2R(:, :, i)*[ddalpha_traj(i); ddbeta_traj(i)] + J_a_Pe(:, :, i)*endAcc_traj(:, i) + ...
                                                dJ_a_2R(:, :, i)*[dalpha_traj(i); dbeta_traj(i)] + dJ_a_Pe(:, :, i)*endVel_traj(:, i);
end

% show test results
disp("IK max error check:"); disp(max(abs(configs_test - configs_expect), [], 2));
disp("velocity max error check:"); disp(max(abs(dconfigs_test - dconfigs_expect), [], 2));
disp("accel max error check:"); disp(max(abs(ddconfigs_test - ddconfigs_expect), [], 2));
