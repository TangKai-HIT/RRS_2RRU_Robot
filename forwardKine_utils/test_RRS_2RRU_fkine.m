%test cases of fkine function

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

%% Test trajectory
wayPoints = [255*1e-3, -deg2rad(25), deg2rad(25);
                        400*1e-3, deg2rad(25), -deg2rad(25)]';
timePoints = [0, 3];
N = 100;
[q,~,~,~] = cubicpolytraj(wayPoints,timePoints,linspace(timePoints(1), timePoints(end), N));

Tf_P = zeros(4,4,N);
Tf_BTC = zeros(4,4,N);
for i=1:N
     [Tf_BTC(:, :, i), Tf_P(:, :, i)] = RRS_2RRU.setEndEffectorSE3(q(1, i), q(2,i), q(3,i));
end

configs = zeros(3, N);
for i=1:N
    configs(:, i) = RRS_2RRU.invKineUpdate(Tf_BTC(:, :, i));
end

%% Test FK
rng(1);

test_fkine_results = zeros(N, 3);
for i=1:N
    %sample around ground truth
    z_p0 = q(1,i) + 20*1e-3*rand;
    alpha0 = q(2,i) + deg2rad(5)*rand;
    beta0 =q(3,i) + deg2rad(5)*rand;
    %get test ik results
    test_fkine_results(i, :) = RRS_2RRU_fkine(configs(:, i), R, r, L1, L2, z_p0, alpha0, beta0);
end

disp("error check on z_p:"); disp(max(abs(q(1, :) - test_fkine_results(:, 1)'), [], 2));
disp("error check on alpha:"); disp(max(abs(q(2, :) - test_fkine_results(:, 2)'), [], 2));
disp("error check on beta:"); disp(max(abs(q(3, :) - test_fkine_results(:, 3)'), [], 2));
