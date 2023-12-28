%test 2RRU_RRS_Hybrid constrained tool tip trajecory kinematics using getAllEndConsKine_Hybrid.m
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
N = 4000;
dt = (timePoints(end) - timePoints(1)) / (N-1);
time_sequence = linspace(timePoints(1), timePoints(end), N);
[q,qd,qdd,~] = cubicpolytraj(wayPoints,timePoints,time_sequence);

alpha_traj = q(4, :);
dalpha_traj = qd(4, :);
ddalpha_traj = qdd(4, :);

beta_traj = q(5, :);
dbeta_traj = qd(5, :);
ddbeta_traj = qdd(5, :);

angle2R_stateTraj = [alpha_traj; beta_traj; dalpha_traj; dbeta_traj; ddalpha_traj; ddbeta_traj];

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

endState_traj = [endPos_traj; endVel_traj; endAcc_traj];

dconfigs_expect = (Diff1_Mat*configs_expect')';
ddconfigs_expect = (Diff2_Mat*configs_expect')';

%% test IK , jacobian, jacobian derivative by given alpha, beta, P
configs_test = zeros(5, N);
dconfigs_test = zeros(5, N);
ddconfigs_test = zeros(5, N);

state_configs_test = zeros(15, N);
dJ_a_2R_test = zeros(5, 2, N);
dJ_a_Pe_test = zeros(5, 3, N);

J_a_2R = zeros(5, 2, N);
J_a_Pe = zeros(5, 3, N);
dJ_a_2R = zeros(5, 2, N);
dJ_a_Pe = zeros(5, 3, N);
J_a_tool = zeros(5, 6, N);
dJ_a_tool = zeros(5, 6, N);
dJ_a_pkm = zeros(3, 6, N);

tic;
for i=1:N
    %class functions
    configs_test(:, i) = Hyd_RRS_2RRU.getEndPosConInvKine2R(endPos_traj(:, i), alpha_traj(i), beta_traj(i));

    J_a_tool(:, :, i) = Hyd_RRS_2RRU.getActuationBodyJacob();
    [J_a_2R(:, :, i), J_a_Pe(:, :, i)] = Hyd_RRS_2RRU.getEndPosConJacobian2R();
    dconfigs_test(:, i) = J_a_2R(:, :, i)*angle2R_stateTraj(3:4,i) + J_a_Pe(:, :, i)*endVel_traj(:, i);

    [dJ_a_2R(:, :, i), dJ_a_Pe(:, :, i)] = Hyd_RRS_2RRU.getEndPosConJacobian2RDiff(dalpha_traj(i), dbeta_traj(i), endVel_traj(:, i));
    ddconfigs_test(:, i) = J_a_2R(:, :, i)*angle2R_stateTraj(5:6,i) + J_a_Pe(:, :, i)*endAcc_traj(:, i) + ...
                                                dJ_a_2R(:, :, i)*angle2R_stateTraj(3:4,i) + dJ_a_Pe(:, :, i)*endVel_traj(:, i);
end
solvetime1 = toc;
fprintf("class functions solve time: %.3f\n", solvetime1);

tic;
for i=1:N
    state_configs_test(:,i) = getAllEndConsKine_Hybrid(angle2R_stateTraj(:,i), endState_traj(:,i), R, r, L1, L2, toolHight, pkm_y_offset, pkm_z_offset);
end
solvetime2 = toc;
fprintf("normal function solve time: %.3f\n", solvetime2);

% show test results
disp("IK max error check:"); disp(max(abs(state_configs_test(1:5,:) - configs_expect), [], 2));
disp("velocity max error check:"); disp(max(abs(state_configs_test(6:10,:) - dconfigs_expect), [], 2));
disp("accel max error check:"); disp(max(abs(state_configs_test(11:15,:) - ddconfigs_expect), [], 2));
