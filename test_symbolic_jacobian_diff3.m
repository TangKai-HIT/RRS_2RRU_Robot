%test3: test 2RRU_RRS_Hybrid joints states changing w.r.t tool angles states, while tool tip states being fixed (using symbolic diff results)
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
            pi/6, d1_init + 0.3, 350*1e-3, pi/8, -pi/7]';
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

angle2R_stateTraj = [alpha_traj; beta_traj; dalpha_traj; dbeta_traj; ddalpha_traj; ddbeta_traj];
angle2R_stateTrajDiff = diff(angle2R_stateTraj, 1, 2);

Tf_tool_path = zeros(4,4,N);
for i=1:N
    Tf_tool_path(:, :, i) = Hyd_RRS_2RRU.forwardKine(q(1, i), q(2,i), q(3,i), q(4,i), q(5,i));
end

endPos_traj = reshape(Tf_tool_path(1:3,4,:), 3, N);
Diff1_Mat = getDiffMatrix_quadInterp(time_sequence, 1);
Diff2_Mat = getDiffMatrix_quadInterp(time_sequence, 2);
endVel_traj = (Diff1_Mat*endPos_traj')';
endAcc_traj = (Diff2_Mat*endPos_traj')';

testId = 506;
testEndState = [endPos_traj(:,testId); endVel_traj(:, testId); endAcc_traj(:, testId)];

config_stateTraj_expect = zeros(5*3, N);
for i=1:N
    config_stateTraj_expect(1:5, i) = Hyd_RRS_2RRU.getEndPosConInvKine2R(testEndState(1:3), alpha_traj(i), beta_traj(i));

    Hyd_RRS_2RRU.getActuationBodyJacob();
    [J_a_2R, J_a_Pe] = Hyd_RRS_2RRU.getEndPosConJacobian2R();
    config_stateTraj_expect(6:10, i) = J_a_2R*[dalpha_traj(i); dbeta_traj(i)] + J_a_Pe*testEndState(4:6);

    [dJ_a_2R, dJ_a_Pe] = Hyd_RRS_2RRU.getEndPosConJacobian2RDiff(dalpha_traj(i), dbeta_traj(i), testEndState(4:6));
    config_stateTraj_expect(11:15, i) = J_a_2R*[ddalpha_traj(i); ddbeta_traj(i)] + J_a_Pe*testEndState(7:9) + ...
                                                dJ_a_2R*[dalpha_traj(i); dbeta_traj(i)] + dJ_a_Pe*testEndState(4:6);
end

config_stateTrajDiff_expect = diff(config_stateTraj_expect, 1, 2);

%% test IK , jacobian, jacobian derivative by given alpha, beta, P
config_stateTrajDiff_test = zeros(5*3, N-1);

% J_a_2R = zeros(5, 2, N-1);
% dJ_a_2R = zeros(5, 2, N-1);

J_Xq_X2R = zeros(15, 6);
tic;
for i=1:N-1
    %Note: numerical accuracy should be lower than 1e-7 when using functions trans from symbolic ones, otherwise NAN would occur
    if abs(alpha_traj(i))<1e-7; alpha_traj(i)=sign(alpha_traj(i))*1e-7; end
    if abs(beta_traj(i))<1e-7; beta_traj(i)=sign(beta_traj(i))*1e-7; end
    testEndState(abs(testEndState(:))<1e-7) = sign(testEndState(abs(testEndState(:))<1e-7))*1e-7;

    J_a_2R= Hyd_Ja2R_2(angle2R_stateTraj(1:2, i)', testEndState(1:3));
    
    dJ_a2R_d2R = Hyd_dJa2R_d2R_2(angle2R_stateTraj(1:2, i)', testEndState(1:3));
    dJ_aPe_d2R = Hyd_dJaPe_d2R_2(angle2R_stateTraj(1:2, i)', testEndState(1:3));
    H1 = tensorprod(dJ_a2R_d2R, angle2R_stateTraj(3:4, i), 2, 1) + tensorprod(dJ_aPe_d2R, testEndState(4:6), 2, 1);
    H2 = H1 + tensorprod(dJ_a2R_d2R, angle2R_stateTraj(3:4, i), 3, 1);

    ddJ_a2R_dd2R = Hyd_ddJa2R_dd2R_2(angle2R_stateTraj(1:2, i)', testEndState(1:3));
    ddJ_aPe_dd2R = Hyd_ddJaPe_dd2R_2(angle2R_stateTraj(1:2, i)', testEndState(1:3));
    A = tensorprod(dJ_a2R_d2R, angle2R_stateTraj(5:6, i), 2, 1) + tensorprod(dJ_aPe_d2R, testEndState(7:9), 2, 1) + ...
            tensorprod(tensorprod(ddJ_a2R_dd2R, angle2R_stateTraj(3:4, i), 2, 1), angle2R_stateTraj(3:4, i), 2, 1) + ...
            tensorprod(tensorprod(ddJ_aPe_dd2R, testEndState(4:6), 2, 1), angle2R_stateTraj(3:4, i), 2, 1);
    
    J_Xq_X2R(1:5, 1:2) = J_a_2R;
    J_Xq_X2R(6:10, 1:2) = H1;
    J_Xq_X2R(6:10, 3:4) = J_a_2R;
    J_Xq_X2R(11:15, 1:2) = A;
    J_Xq_X2R(11:15, 3:4) = H2;
    J_Xq_X2R(11:15, 5:6) = J_a_2R;

    config_stateTrajDiff_test(:, i) = J_Xq_X2R * angle2R_stateTrajDiff(:, i);
end
runtime1 = toc;
fprintf("symbolic diff function run time: %.3f\n", runtime1);

% show test results
disp("error check:"); disp(max(abs(config_stateTrajDiff_test - config_stateTrajDiff_expect), [], 2));
