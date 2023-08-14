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
    [q_inv(1,i), q_inv(2,i), theta_pkm(:, i)] = Hyd_RRS_2RRU.invKine(Tf_tool_path(:, :, i));
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