%test_2RRU_RRS_Hybrid

%% Parameters & init
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

%% Test trajectory
wayPoints = [-pi/6, d1_init, 220*1e-3, 0, 0;
            pi/6, d1_init + 0.3, 250*1e-3, pi/8, -pi/8]';
timePoints = [0, 3];
N = 500;
dt = (timePoints(end) - timePoints(1)) / (N-1);
[q,qd,qdd,~] = cubicpolytraj(wayPoints,timePoints,linspace(timePoints(1), timePoints(end), N));

Tf_tool_path = zeros(4,4,N);
for i=1:N
    Tf_tool_path(:, :, i) = Hyd_RRS_2RRU.forwardKine(q(1, i), q(2,i), q(3,i), q(4,i), q(5,i));
end

%% Check invIK
q_inv = zeros(size(q));
theta_pkm = zeros(3, N);
J_a_tool_b = zeros(5, 6, N);
J_a_P_b = zeros(5, 6, N);

for i=1:N
    config = Hyd_RRS_2RRU.invKine(Tf_tool_path(:, :, i));
    q_inv(1,i) = config(1); 
    q_inv(2,i) = config(2); 
    theta_pkm(:, i) = config(3:5); 
    q_inv(3,i) = Hyd_RRS_2RRU.RRS_2RRU.z_p;
    q_inv(4,i) = Hyd_RRS_2RRU.RRS_2RRU.alpha;
    q_inv(5,i) = Hyd_RRS_2RRU.RRS_2RRU.beta;
    [J_a_tool_b(:, :, i), J_a_P_b(:, :, i)] = Hyd_RRS_2RRU.getActuationBodyJacob();
end

%% Check Jacobian
%joints velocity by Jacobian
dX_tool_b = zeros(6,N-1);
dq_Jacob = zeros(5,N-1);

for i=1:N-1
    dX_tool_b(:, i) = logMapSE3(Tf_tool_path(:, :, i) \ Tf_tool_path(:, :, i+1)) / dt;
    dq_Jacob(:, i) = J_a_tool_b(:, :, i) * dX_tool_b(:, i);
end

%joints velocity by diff
d_theta_pkm = diff(theta_pkm, 1, 2)/dt;
d_q_diff = [diff(q_inv(1:2, :), 1, 2)/dt; d_theta_pkm];

%% Check Singularity (plot surface)
N = 100;
alpha_space = linspace(-deg2rad(25), deg2rad(25), N);
beta_space = linspace(-deg2rad(25), deg2rad(25), N);
z_p = 330*1e-3;

[alpha_grid, beta_grid] = meshgrid(rad2deg(alpha_space), rad2deg(beta_space));

fkine_condNum_grid = zeros(N, N);
invkine_condNum_grid = zeros(N, N);
for i = 1:N
    for j = 1:N
        cur_alpha = alpha_space(i);
        cur_beta = beta_space(j);
        
        [Tf_BTC, ~] = Hyd_RRS_2RRU.RRS_2RRU.setEndEffectorSE3(z_p, cur_alpha, cur_beta);
        Hyd_RRS_2RRU.RRS_2RRU.invKineUpdate(Tf_BTC);
        [fkine_condNum_grid(i, j), invkine_condNum_grid(i, j)] = Hyd_RRS_2RRU.RRS_2RRU.manipulationCondNum();
    end
end

fprintf("min fkine manipulability condition number:\n %.2f\n", min(fkine_condNum_grid, [], 'all'));
fprintf("min invkine manipulability condition number:\n %.2f\n", min(invkine_condNum_grid, [], 'all'));

f = figure("Position",[200, 100, 1500, 800]);
subplot(1,2,1);
surf(alpha_grid, beta_grid, fkine_condNum_grid);
title("fkine manipulability condition number");
xlabel("alpha/°"); ylabel("beta/°");

subplot(1,2,2);
surf(alpha_grid, beta_grid, invkine_condNum_grid);
title("invkine manipulability condition number");
xlabel("alpha/°"); ylabel("beta/°");

sgtitle(sprintf("At operation height z_p=%.2f m", z_p));