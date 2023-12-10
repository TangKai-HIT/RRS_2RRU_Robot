%测算并联机构工作空间中的关节角范围
%初始化
R = 104*1e-3;
r = 104*1e-3;
L1 = 208*1e-3;
L2 = 249.6*1e-3; 
toolHight = 136.8*1e-3;
z_p_min = 201.937*1e-3;

RRS_2RRU = RRS_2RRU_Basic(R, r, L1, L2, toolHight, z_p_min);

%离散参数
zp_range = [RRS_2RRU.z_p_min, 0.4];
alpha_range = [-pi/6,pi/6]; beta_range = [-pi/6,pi/6];
% alpha_range = [-deg2rad(25),deg2rad(25)]; beta_range = [-deg2rad(25),deg2rad(25)];
N_zp = 10;
N = 100;

zp_space = linspace(zp_range(1), zp_range(2), N_zp);
alpha_space = linspace(alpha_range(1), alpha_range(2), N);
beta_space = linspace(beta_range(1), beta_range(2), N);

%奇异位型
singular_tol = 0.02; %singularity check tolerance
cond_tol = 0.02; %condition number check tolerance

%% 测试各个关节转角范围
%关节角度范围
thetaRan_A1B1 = repmat([inf, -inf], N_zp, 1);
thetaRan_A2B2 = repmat([inf, -inf], N_zp, 1);
thetaRan_A3B3 = repmat([inf, -inf], N_zp, 1);

thetaRan_B1C1 = repmat([inf, -inf], N_zp, 1);
thetaRan_B2C2 = repmat([inf, -inf], N_zp, 1);
thetaRan_B3C3 = repmat([inf, -inf], N_zp, 1);

thetaRan_U11 = repmat([inf, -inf], N_zp, 1);
thetaRan_U12 = repmat([inf, -inf], N_zp, 1);
thetaRan_U21 = repmat([inf, -inf], N_zp, 1);
thetaRan_S = repmat([inf, -inf], N_zp, 1);

%球副斜面法向量s_n在P坐标系下
% s_n_P = eul2rotm([0,0,-pi/6], "ZYX") * [0; -1; 0]; %球铰30度安装
% s_n_P = eul2rotm([0,0,-deg2rad(40)], "ZYX") * [0; -1; 0]; %球铰40度安装
s_n_P = eul2rotm([0,0,-pi/4], "ZYX") * [0; -1; 0]; %球铰45度安装
% s_n_P = eul2rotm([0,0,-deg2rad(44)], "ZYX") * [0; -1; 0]; %球铰50度安装
% s_n_P = eul2rotm([0,0,-deg2rad(60)], "ZYX") * [0; -1; 0]; %球铰60度安装
% s_n_P = eul2rotm([0,0,-deg2rad(70)], "ZYX") * [0; -1; 0]; %球铰70度安装

%分层计算
for i = 1:N_zp
    cur_zp = zp_space(i);
    for cur_alpha = alpha_space
        for cur_beta = beta_space
            %set pose & IK
            [Tf_BTC, ~] = RRS_2RRU.setEndEffectorSE3(cur_zp, cur_alpha, cur_beta);
            thetas = RRS_2RRU.invKineUpdate(Tf_BTC);   

            %skip non valid solutions
            if isempty(thetas)
                continue;
            end

            %singularity condition
            [forwardSingular, inverseSingular] = RRS_2RRU.checkSingularity(singular_tol);

            %condition number condition
            J_a = RRS_2RRU.getActuationJacob();
            [~, J_r] = RRS_2RRU.getOutputJacob();
            J = J_a*J_r;
            cond0 = (1/cond(J)) > cond_tol;

            %get max range of joint angles
            if ~(forwardSingular || inverseSingular) && cond0
                theta_A1B1 = thetas(1);
                theta_A2B2 = pi - thetas(2);
                theta_A3B3 = thetas(3);
                theta_B1C1 = acos(dot(-RRS_2RRU.b1, RRS_2RRU.c11));
                theta_B2C2 = acos(dot(-RRS_2RRU.b2, RRS_2RRU.c21));
                theta_B3C3 = acos(dot(-RRS_2RRU.b3, RRS_2RRU.c31));
                theta_U11 = acos(dot(RRS_2RRU.c11, RRS_2RRU.c12));
                theta_U21 = acos(dot(RRS_2RRU.c21, RRS_2RRU.c22));

                s_n = RRS_2RRU.R_P * s_n_P;
                theta_S = acos(dot(s_n, RRS_2RRU.c31));

                thetaRan_A1B1(i, 1) = min(thetaRan_A1B1(i, 1), rad2deg(theta_A1B1));
                thetaRan_A1B1(i, 2) = max(thetaRan_A1B1(i, 2), rad2deg(theta_A1B1));

                thetaRan_A2B2(i, 1) = min(thetaRan_A2B2(i, 1), rad2deg(theta_A2B2));
                thetaRan_A2B2(i, 2) = max(thetaRan_A2B2(i, 2), rad2deg(theta_A2B2));

                thetaRan_A3B3(i, 1) = min(thetaRan_A3B3(i, 1), rad2deg(theta_A3B3));
                thetaRan_A3B3(i, 2) = max(thetaRan_A3B3(i, 2), rad2deg(theta_A3B3));

                thetaRan_B1C1(i, 1) = min(thetaRan_B1C1(i, 1), rad2deg(theta_B1C1));
                thetaRan_B1C1(i, 2) = max(thetaRan_B1C1(i, 2), rad2deg(theta_B1C1));

                thetaRan_B2C2(i, 1) = min(thetaRan_B2C2(i, 1), rad2deg(theta_B2C2));
                thetaRan_B2C2(i, 2) = max(thetaRan_B2C2(i, 2), rad2deg(theta_B2C2));

                thetaRan_B3C3(i, 1) = min(thetaRan_B3C3(i, 1), rad2deg(theta_B3C3));
                thetaRan_B3C3(i, 2) = max(thetaRan_B3C3(i, 2), rad2deg(theta_B3C3));

                thetaRan_U11(i, 1) = min(thetaRan_U11(i, 1), rad2deg(theta_U11));
                thetaRan_U11(i, 2) = max(thetaRan_U11(i, 2), rad2deg(theta_U11));

                thetaRan_U12(i, 1) = min(thetaRan_U12(i, 1), rad2deg(cur_alpha));
                thetaRan_U12(i, 2) = max(thetaRan_U12(i, 2), rad2deg(cur_alpha));

                thetaRan_U21(i, 1) = min(thetaRan_U21(i, 1), rad2deg(theta_U21));
                thetaRan_U21(i, 2) = max(thetaRan_U21(i, 2), rad2deg(theta_U21));

                thetaRan_S(i, 1) = min(thetaRan_S(i, 1), rad2deg(theta_S));
                thetaRan_S(i, 2) = max(thetaRan_S(i, 2), rad2deg(theta_S));
            end
        end
    end
end

%% 优化球铰的基座安装角度：以zp=0.25~0.4m, alpha,beta=-30°~30°范围内有效工作空间（归一化尺度）的体积来优化基座安装角度
zp_range = [0.25, 0.4];
N_zp = 50;
zp_space = linspace(zp_range(1), zp_range(2), N_zp);

angle_space = 30:70;

maxk_valid = cell(1, length(angle_space));
for i=1:length(angle_space)
    maxk_valid{i} = zeros(N, N, N_zp);
end
volumes = zeros(1, length(angle_space));

for i = 1:N_zp
    cur_zp = zp_space(i);
    for j = 1:N
        cur_alpha = alpha_space(j);
        for k = 1:N
            cur_beta = beta_space(k);
            %set pose & IK
            [Tf_BTC, ~] = RRS_2RRU.setEndEffectorSE3(cur_zp, cur_alpha, cur_beta);
            thetas = RRS_2RRU.invKineUpdate(Tf_BTC);   

            %skip non valid solutions
            if isempty(thetas)
                continue;
            end

            %singularity condition
            [forwardSingular, inverseSingular] = RRS_2RRU.checkSingularity(singular_tol);

            %condition number condition
            J_a = RRS_2RRU.getActuationJacob();
            [~, J_r] = RRS_2RRU.getOutputJacob();
            J = J_a*J_r;
            cond0 = (1/cond(J)) > cond_tol;

            %get max range of joint angles
            if ~(forwardSingular || inverseSingular) && cond0
                %check ball joint angle
                for n = 1:length(angle_space) 
                    s_n_P = eul2rotm([0,0,-deg2rad(angle_space(n))], "ZYX") * [0; -1; 0];
                    s_n = RRS_2RRU.R_P * s_n_P;
                    theta_S = acos(dot(s_n, RRS_2RRU.c31));

                    if theta_S<=pi/6 %record points within 30 degree of ball joint angle
                        maxk_valid{n}(j, k, i) = 1;
                    end
                end
            end

        end
    end
end

%compute volumes
for n = 1:length(angle_space) 
    area = zeros(1, N_zp);
    for k = 1:N_zp
        area(k) = trapz(trapz(maxk_valid{n}(:, :, k), 1));
    end
    volumes(n) = trapz(area);
end

%max volume angle
[maxVol, maxId] = max(volumes);
disp("max workspace volume install angle:"); disp(angle_space(maxId));