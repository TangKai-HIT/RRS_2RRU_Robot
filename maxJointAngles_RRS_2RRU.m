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
alpha_range = [-pi/6,pi/6];
beta_range = [-pi/6,pi/6];
N_zp = 8;
N = 20;

zp_space = linspace(zp_range(1), zp_range(2), N_zp);
alpha_space = linspace(alpha_range(1), alpha_range(2), N);
beta_space = linspace(beta_range(1), beta_range(2), N);

%奇异位型
singular_tol = 0.02; %singularity check tolerance
cond_tol = 0.02; %condition number check tolerance

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
thetaRan_S = inf;

%球副斜面法向量s_n在P坐标系下
s_n_P = eul2rotm([0,0,-pi/4], "ZYX") * [0; -1; 0];

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
                theta_S = pi/2 - acos(dot(s_n, RRS_2RRU.c31));

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

                thetaRan_S = min(thetaRan_S, rad2deg(theta_S));
            end
        end
    end
end

