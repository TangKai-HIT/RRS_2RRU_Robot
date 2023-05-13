classdef RRS_2RRU_Basic < handle
    %2RRU_RRS_ROBOT Summary of this class goes here
    %   Detailed explanation goes here

    properties
        R;
        r;
        L1;
        L2;
        toolHight; %cutter BTC hight (z) in P frame
        thetas;
    end

    properties(SetAccess=private, GetAccess=public)
        %inverse kinematic points
        P;
        C1;
        C2;
        C3;
        B1;
        B2;
        B3;
        A1;
        A2;
        A3;
        R_P;
        TC; %tool center

        %max & min z_p, operation height, route on z_p
        z_p_max;
        z_p_min;
        z_p_operation;
        delta_z_p;

        %independent motion coordinate
        z_p; alpha; beta;

        %dirctional vectors
        c11; c12;
        c21; c22;
        c31; c32;
        b1; b2; b3;
    end

    methods
        function obj = RRS_2RRU_Basic(R, r, L1, L2, toolHight, z_p_min)
            %RRS_2RRU_ROBOT Construct an instance of this class
            %   Detailed explanation goes here
            if nargin>0
                obj.R = R;
                obj.r = r;
                obj.L1 = L1;
                obj.L2 = L2;
                obj.toolHight = toolHight;
                obj.A1 = [obj.R; 0; 0];
                obj.A2 = [-obj.R; 0; 0];
                obj.A3 = [0; obj.R; 0];
                obj.z_p_max = sqrt((L1+L2)^2-(R-r)^2);
                obj.z_p_min = z_p_min;
                obj.z_p_operation = (obj.z_p_max + obj.z_p_min) / 2;
                obj.delta_z_p = obj.z_p_max - obj.z_p_min;
            end
        end

        function thetas = invKineUpdate(obj, Tf_BTC)
            %INVKINEUPDATE inverse kinematic of the robot given SE3 of P(end-effector)
            %   Inputs:
            %       Tf_BTC: SE3 of BTC w.r.t base O
            thetas = zeros(1,3);
            
            btc_p = [0, 0, obj.toolHight];
            Tf_p_btc = trvec2tform(btc_p);
            
            obj.TC = Tf_BTC(1:3, 4);
            obj.R_P = Tf_BTC(1:3, 1:3);

            Tf_P = Tf_BTC * getInvSE3(Tf_p_btc);
            obj.P = Tf_P(1:3, 4);

            C1_p = [obj.r; 0; 0];
            C2_p = [-obj.r; 0; 0];
            C3_p = [0; obj.r; 0];

            C1_hom = Tf_P * [C1_p; 1];
            C2_hom = Tf_P * [C2_p; 1];
            C3_hom = Tf_P * [C3_p; 1];

            obj.C1 = C1_hom(1:3);
            obj.C2 = C2_hom(1:3);
            obj.C3 = C3_hom(1:3);

            vec_A1C1 = obj.C1 - obj.A1;
            vec_A2C2 = obj.C2 - obj.A2;
            vec_A3C3 = obj.C3 - obj.A3;
            
            %theta11(R-R-R crank)
            delta = atan2(vec_A1C1(3), vec_A1C1(1));
            d = norm(vec_A1C1);
            gamma = acos((d^2 + obj.L1^2 - obj.L2^2)/(2*d*obj.L1));
            thetas(1) = delta - gamma;
            
            %theta12(R-R-R crank)
            delta = atan2(vec_A2C2(3), vec_A2C2(1));
            d = norm(vec_A2C2);
            gamma = acos((d^2 + obj.L1^2 - obj.L2^2)/(2*d*obj.L1));
            thetas(2) = gamma + delta;

            %theta13(R-R-R crank)
            delta = atan2(vec_A3C3(3), vec_A3C3(2));
            d = norm(vec_A3C3);
            gamma = acos((d^2 + obj.L1^2 - obj.L2^2)/(2*d*obj.L1));
            thetas(3) = delta - gamma;
            
            %return empty if get nonvalid solution
            if ~isreal(thetas)
                thetas = [];
                obj.thetas = [];
                return;
            end

            %record result
            obj.thetas = thetas;

            %update Bi
            obj.B1 = obj.A1 + eul2rotm([0, -thetas(1), 0], "ZYX") * [obj.L1; 0; 0];
            obj.B2 = obj.A2 + eul2rotm([0, -thetas(2), 0], "ZYX") * [obj.L1; 0; 0];
            obj.B3 = obj.A3 + eul2rotm([0, 0, thetas(3)], "ZYX") * [0; obj.L1; 0];

            %update dirctional vectors
            obj.b1 = (obj.B1 - obj.A1)/obj.L1;
            obj.b2 = (obj.B2 - obj.A2)/obj.L1;
            obj.b3 = (obj.B3 - obj.A3)/obj.L1;

            obj.c11 = (obj.C1 - obj.B1)/obj.L2;
            obj.c21 = (obj.C2 - obj.B2)/obj.L2;
            obj.c31 = (obj.C3 - obj.B3)/obj.L2;

            obj.c12 = (obj.C1 - obj.P)/obj.r;
            obj.c22 = (obj.C2 - obj.P)/obj.r;
            obj.c32 = (obj.C3 - obj.P)/obj.r;
        end
        
        function J_a = getActuationJacob(obj)
            %GETACTUATIONJACOB J_a*dX_p = [dtheta11; dtheta21; dtheta31]

            [J_theta_a, J_X_a] = obj.getSplitedActuationJacob();

            J_a = J_theta_a \ J_X_a;
        end

        function [J_theta_a, J_X_a] = getSplitedActuationJacob(obj)
            %GETSPLITEDACTUATIONJACOB J_theta_a * dtheta = J_X_a * dX_a

            s1 = [0;-1;0]; %theta1
            s2 = [0;-1;0]; %theta2
            s3 = [1;0;0]; %theta3

            J_theta_a = obj.L1 * blkdiag(dot(cross(obj.b1,obj.c11), s1), dot(cross(obj.b2,obj.c21), s2), dot(cross(obj.b3,obj.c31), s3));
            
            J_X_1a = [obj.r*(cross(obj.c12,obj.c11)'), obj.c11'];
            J_X_2a = [obj.r*(cross(obj.c22,obj.c21)'), obj.c21'];
            J_X_3a = [obj.r*(cross(obj.c32,obj.c31)'), obj.c31'];
            
            J_X_a = [J_X_1a; J_X_2a; J_X_3a];
        end
        
        function [forwardSingular, inverseSingular] = checkSingularity(obj, tol)
            %CHECKSINGULARITY check inverse and forward singularity
            forwardSingular = false;
            inverseSingular = false;

            %check forward singularity
            forward = [norm(cross(obj.b1,obj.c11)), norm(cross(obj.b2,obj.c21)), norm(cross(obj.b3,obj.c31))];
            if min(forward) < tol
                forwardSingular = true;
            end
            
            %check inverse singularity
            inverse = [norm(cross(obj.c12,obj.c11)), norm(cross(obj.c22,obj.c21)), norm(cross(obj.c32,obj.c31))];
            if min(inverse) < tol
                inverseSingular = true;
            end
        end

        function thetas = setToInitPose(obj)
            %SETTOINITPOSE set To Initial Pose defined by z_p_min
            P_pose = [obj.z_p_min, 0, 0]; %[z_p, alpha, beta]
            [Tf_BTC, ~] = obj.setEndEffectorSE3(P_pose(1), P_pose(2), P_pose(3));
            thetas = obj.invKineUpdate(Tf_BTC); %invkine
        end

        function R_p = calEndEffectorSO3(obj, alpha, beta)
            %CALENDEFFECTORSE3
            R_p = [cos(beta),  sin(alpha)*sin(beta),  cos(alpha)*sin(beta);
                    0   ,       cos(alpha),         -sin(alpha);
                -sin(beta), sin(alpha)*cos(beta),  cos(alpha)*cos(beta)];
        end

        function [Tf_BTC, Tf_P] = setEndEffectorSE3(obj, zp, alpha, beta)
            %CALENDEFFECTORSE3
            obj.z_p = zp;
            obj.alpha = alpha;
            obj.beta = beta;

            R_p = obj.calEndEffectorSO3(alpha, beta);

            P_0 = [-sin(alpha)*sin(beta)*obj.r; 0; zp];

            Tf_P = [R_p, P_0;
                    zeros(1,3),  1];

            btc_p = [0, 0, obj.toolHight];
            Tf_BTC = Tf_P * trvec2tform(btc_p);
        end

        function [J_rt, J_rp] = getOutputJacob(obj, alpha, beta)
            %GETOUTPUTJACOB J_rp*[dz_p; dalpha, dbeta] = dX_p
            if ~exist("alpha","var") && ~exist("beta","var")
                alpha = obj.alpha;
                beta = obj.beta;
            end

            J_rp_T = [0, -cos(alpha)*sin(beta)*obj.r, -sin(alpha)*cos(beta)*obj.r;
                      0,           0,                           0;
                      1,            0,                          0];
            
            J_rp_R = [0,    cos(beta),  0;
                      0,      0,        1;
                      0,    -sin(beta), 0];

            J_rp = [J_rp_R; J_rp_T];
            
            R_p = obj.calEndEffectorSO3(alpha, beta);
            r_tool = R_p * [0; 0; obj.toolHight];
            trans = [eye(3),        zeros(3);
                    -skewMatrix(r_tool),     eye(3)];
            
            J_rt = trans * J_rp;
        end

        function torque_actuate = getStaticForce_tool(obj, torque_body, force_body)
            %GETSTATICFORCE_TOOL get equilibrium torque on actuators when setting wrench on tool center 
            %   Inputs:
            %       torque_body, force_body: 3 X 1, torque & force in local BTC frame

            J_a = obj.getActuationJacob();
            wrench_tool = [torque_body; force_body]; %wrench in tool center frame
            
            %transform (adjoint map to P frame that aligned with base frame)
            Tf_Pa_BTC = [obj.R_P,    obj.R_P*[0; 0; obj.toolHight];
                                    zeros(1,3),           1];

            wrench_P = (adjointMatrix(getInvSE3(Tf_Pa_BTC))') * wrench_tool;
            
            [~, J_rp] = obj.getOutputJacob();

            torque_actuate = - (J_a*J_rp)' \ (J_rp' *  wrench_P);
        end

        function f = plotToolWorkSpace(obj, zp_range, alpha_range, beta_range, N_zp)
            %PLOTTOOLWORKSPACE plot tool tip workspace
            N = 100;
            zp_space = linspace(zp_range(1), zp_range(2), N_zp);
            alpha_space = linspace(alpha_range(1), alpha_range(2), N);
            beta_space = linspace(beta_range(1), beta_range(2), N);
                     
            f = figure();
            
            singular_tol = 0.02; %singularity check tolerance
            cond_tol = 0.02; %condition number check tolerance

            for cur_zp = zp_space
                x_tool = [];
                y_tool = [];
                z_tool = [];
                %surface on zp_i
                for cur_alpha = alpha_space
                    for cur_beta = beta_space
                        %set pose & IK
                        [Tf_BTC, ~] = obj.setEndEffectorSE3(cur_zp, cur_alpha, cur_beta);
                        obj.invKineUpdate(Tf_BTC);
                        
                        %skip non valid solutions
                        if isempty(obj.thetas)
                            continue;
                        end

                        %singularity condition
                        [forwardSingular, inverseSingular] = obj.checkSingularity(singular_tol);

                        %condition number condition
                        J_a = obj.getActuationJacob();
                        [~, J_r] = obj.getOutputJacob();
                        J = J_a*J_r;
                        cond0 = (1/cond(J)) > cond_tol;

                        %other conditions
%                         cond1 = 

                        %add new points
                        if ~(forwardSingular || inverseSingular) && cond0
                            x_tool = [x_tool, obj.TC(1)];
                            y_tool = [y_tool,obj.TC(2)];
                            z_tool = [z_tool,obj.TC(3)];
                        end
                    end
                end

                %plot layer on zp_i
                if ~isempty(x_tool)
                    [X_tool,Y_tool]=meshgrid(linspace(min(x_tool),max(x_tool),N), linspace(min(y_tool),max(y_tool),N));
                    Z_tool=griddata(x_tool, y_tool, z_tool, X_tool, Y_tool);

                    surf(X_tool,Y_tool,Z_tool,'LineStyle','none');
                    colormap jet; shading interp; alpha 0.7;
                    colorbar;

                    set(gca,'Fontname','times new Roman','Fontsize',10);
            %         set(gca,'XLim',[-150 150],'YLim',[-150 150],'ZLim',[200 700]);
            %         set(gca,'XTick',[-150:75:150],'YTick',[-150:75:150],'ZTick',[200:100:700]);
                    xlabel('Tool_x/m'),ylabel('Tool_y/m'); zlabel('Tool_z/m');
                    hold on
                end
            end
        end

    end
end