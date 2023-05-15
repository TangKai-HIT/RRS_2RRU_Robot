function [f, maxActInfo] = plotMaxActuationSpace(RRS_2RRU, wrench, zp_range, alpha_range, beta_range, N_zp)
%plotMaxActuationSpace plot actuation workspace w.r.t exerted force
%   Inputs: 
%       wrench: 6 X num

%% init
N = 100;
zp_space = linspace(zp_range(1), zp_range(2), N_zp);
alpha_space = linspace(alpha_range(1), alpha_range(2), N);
beta_space = linspace(beta_range(1), beta_range(2), N);

singular_tol = 0.02; %singularity check tolerance
cond_tol = 0.02; %condition number check tolerance

maxActInfo.z_p_space = zp_space;
maxActInfo.max_actuations = zeros(3, N_zp); 
maxActInfo.poses_a1 = zeros(3, N_zp); 
maxActInfo.poses_a2 = zeros(3, N_zp);
maxActInfo.poses_a3 = zeros(3, N_zp);

%% generate layers
f(1) = figure(Position=[150,100, 1200, 900]);
f(2) = figure(Position=[150,100, 1200, 900]);
f(3) = figure(Position=[150,100, 1200, 900]);

for i = 1:N_zp
    cur_zp = zp_space(i);
%     x_tool = [];
%     y_tool = [];
%     z_tool = [];
    alpha = [];
    beta = [];
    max_actuations = [];
    %surface on zp_i
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

            %other conditions
%                         cond1 = 

            %add new points
            if ~(forwardSingular || inverseSingular) && cond0
%                 x_tool = [x_tool, RRS_2RRU.TC(1)];
%                 y_tool = [y_tool,RRS_2RRU.TC(2)];
%                 z_tool = [z_tool,RRS_2RRU.TC(3)];
                alpha = [alpha, cur_alpha];
                beta = [beta, cur_beta];

                max_act = [0; 0; 0];
                act_G = RRS_2RRU.getEquGravityForce(); %actuation against gravity 
                for n = 1:size(wrench, 2)
                    cur_act = abs(RRS_2RRU.getStaticForce_tool(wrench(1:3, n), wrench(4:6, n)) + act_G);

                    index = find(max_act<cur_act);
                    max_act(index) = cur_act(index);
                end
                max_actuations = [max_actuations, max_act];

                %record max actuation pose on this layer
                if max_act(1) > maxActInfo.max_actuations(1, i)
                    maxActInfo.max_actuations(1, i) = max_act(1);
                    maxActInfo.poses_a1(:, i) = [cur_zp, cur_alpha, cur_beta];
                end

                if max_act(2) > maxActInfo.max_actuations(2, i)
                    maxActInfo.max_actuations(2, i) = max_act(2);
                    maxActInfo.poses_a2(:, i) = [cur_zp, cur_alpha, cur_beta];
                end

                if max_act(3) > maxActInfo.max_actuations(3, i)
                    maxActInfo.max_actuations(3, i) = max_act(3);
                    maxActInfo.poses_a3(:, i) = [cur_zp, cur_alpha, cur_beta];
                end
            end

        end
    end

    %plot contours on zp_i
    if ~isempty(beta)
%         [X_tool,Y_tool]=meshgrid(linspace(min(x_tool),max(x_tool),N), linspace(min(y_tool),max(y_tool),N));
%         Z_tool=griddata(x_tool, y_tool, z_tool, X_tool, Y_tool);
        [Beta,Alpha]=meshgrid(linspace(min(beta),max(beta),N), linspace(min(alpha),max(alpha),N));

        Actuate{1}=griddata(beta, alpha, max_actuations(1, :), Beta, Alpha);
        Actuate{2}=griddata(beta, alpha, max_actuations(2, :), Beta, Alpha);
        Actuate{3}=griddata(beta, alpha, max_actuations(3, :), Beta, Alpha);

        for n = 1:3
            figure(f(n));
            
            subplot(2, ceil(N_zp/2), i);
            contour(rad2deg(Beta), rad2deg(Alpha), Actuate{n});
            colormap cool; shading interp;
            c = colorbar;
            c.Label.String =  "Torque(N*m)";
            title(sprintf("$\\theta_%d,\\quad z_p=%.1f mm$", n, cur_zp*1e3), "Interpreter","latex");
    
%             set(gca,'Fontname','times new Roman','Fontsize',10);
            xlabel('\beta (°)');
            ylabel('\alpha (°)');
        end
    end
end
