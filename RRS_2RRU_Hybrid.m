classdef RRS_2RRU_Hybrid < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here

    properties
        RRS_2RRU;

    end

    properties(SetAccess=private, GetAccess=public)
        d1 = 0;
        theta1 = 0;
        d1_init = 0;
        z_p_init = 0;

        pkm_y_offset = 0; %design param, y axis offset from rotational center in serial part
        pkm_z_offset = 0; %design param, z axis offset from rotational center in serial part  
        
        twist_1 = [1; 0; 0; 0; 0; 0]; %twist of first serial: rotational joint
        twist_2 = [0; 0; 0; 1; 0; 0]; %twist of second serial: prismatic joint

        Tf_BTC_local_init;
        Tf_BTC_init;
        Tf_P_local_init;
        Tf_P_init;

        Tf_P; %P in base coordinate
        Tf_BTC; %BTC in base coordinate
        Tf_P_tool;
        Ad_P_tool; %adjoint matrix of Tf_P_tool

        Tf_C3_P;
        Ad_C3_P;

        Tf_0_P0;
        Tf_0_pkm;
        Tf_pkm_P;

        %jacobians
        J_a_tool; %actuation body jacobian of tool frame
        J_a_P; %actuation body jacobian of tool frame P frame
        J_Opkm_P; % pkm to O frame jacobian in P frame
        J_pkmP_P; %P to pkm frame jacobian in P frame        
    end

    methods
        function obj = RRS_2RRU_Hybrid(RRS_2RRU, z_p_init, d1_init, pkm_y_offset, pkm_z_offset)
            %UNTITLED Construct an instance of this class
            %   Detailed explanation goes here
            if nargin > 0
                obj.RRS_2RRU = RRS_2RRU;
                obj.d1_init = d1_init;
                obj.d1 = d1_init;
                obj.z_p_init = z_p_init;
                obj.pkm_y_offset = pkm_y_offset;
                obj.pkm_z_offset = pkm_z_offset;
                
                [obj.Tf_BTC_local_init, obj.Tf_P_local_init] = obj.RRS_2RRU.setEndEffectorSE3(z_p_init, 0, 0);
                obj.RRS_2RRU.invKineUpdate(obj.Tf_BTC_local_init);

                Tf_0_pkm = expMapSE3(obj.twist_1, obj.theta1) * expMapSE3(obj.twist_2, d1_init) * trvec2tform([0, obj.pkm_y_offset, obj.pkm_z_offset]);
                obj.Tf_P_init = Tf_0_pkm * obj.Tf_P_local_init;
                obj.Tf_P = obj.Tf_P_init;
                obj.Tf_BTC_init = Tf_0_pkm * obj.Tf_BTC_local_init;
                obj.Tf_BTC = obj.Tf_BTC_init;

                obj.Tf_P_tool = getInvSE3(obj.Tf_P) * obj.Tf_BTC;
                obj.Ad_P_tool = adjointMatrix(obj.Tf_P_tool);

                obj.Tf_C3_P = trvec2tform([0,-obj.RRS_2RRU.r, 0]);
                obj.Ad_C3_P = adjointMatrix(obj.Tf_C3_P);
            end
        end
        
        %% general kinematics: 

        function Tf_tool = forwardKine(obj, theta1, d1, z_p, alpha, beta)
            Tf_pkm = expMapSE3(obj.twist_1, theta1) * expMapSE3(obj.twist_2, d1) * trvec2tform([0, obj.pkm_y_offset, obj.pkm_z_offset]);
            [Tf_pkm_tool, ~] = obj.RRS_2RRU.setEndEffectorSE3(z_p, alpha, beta);
            Tf_tool = Tf_pkm*Tf_pkm_tool;
        end
            
        function config = invKine(obj,Tf_BTC)
            %INVKINE inverse kinematic of hybrid robot
            %   output: 
            %       config: 1 x 5, [theta1, d1, thetas_pkm]
            
            obj.Tf_BTC = Tf_BTC;
            %P pose & position in base coordinate
            Tf_BTC_P = trvec2tform([0 0 -obj.RRS_2RRU.toolHight]);
            obj.Tf_P = Tf_BTC * Tf_BTC_P;
            P = obj.Tf_P(1:3, 4);
            
            %solve subproblem for theta1 and z_p
            z_p = sqrt(P(2)^2 + P(3)^2 - obj.pkm_y_offset^2) - obj.pkm_z_offset;
            P0 = [P(1); 0; z_p] + [0; obj.pkm_y_offset; obj.pkm_z_offset];
            obj.theta1 = subIK_rotateOneAxis(obj.twist_1, P0, P);

            %solve subproblem for alpha, beta
            rotm_P = Tf_BTC(1:3, 1:3);
            rotm_P_local = expMapSO3(obj.twist_1(1:3), -obj.theta1) * rotm_P;

            r13 = rotm_P_local(1,3); r33 = rotm_P_local(3,3); r23 = rotm_P_local(2,3);
            alpha = atan2(-r23, sqrt(r13^2 + r33^2));
            beta = atan2(r13, r33); %cos(alpha) ~= 0

            p_x_local = -sin(alpha)*sin(beta)*obj.RRS_2RRU.r;
            obj.d1 = P(1) - p_x_local;
            
            %update Tf_0_P0, Tf_0_pkm
            obj.Tf_0_P0 = expMapSE3(obj.twist_1, obj.theta1) * expMapSE3(obj.twist_2, obj.d1);
            obj.Tf_0_pkm = obj.Tf_0_P0 * trvec2tform([0, obj.pkm_y_offset, obj.pkm_z_offset]);
            obj.Tf_pkm_P =  getInvSE3(obj.Tf_0_pkm) * obj.Tf_P;

            %solve invkine of PKM RRS_2RRU
            [Tf_BTC_local, ~] = obj.RRS_2RRU.setEndEffectorSE3(z_p, alpha, beta); 
            thetas_pkm = obj.RRS_2RRU.invKineUpdate(Tf_BTC_local);

            config = [];
            if ~isempty(thetas_pkm)
                config = [obj.theta1, obj.d1, thetas_pkm];
            end
        end
        
        %% Actuation Jacobians & its derivative: 

        function [J_a_tool, J_a_P, J_a_pkm] = getActuationBodyJacob(obj)
            %GETACTUATIONBODYJACOB get actuation body jacobian from tool tip/P-Point body twist to generalized coordinates, 
            %                   J_a_tool*dX_b_tool = J_a_P*dX_b_P = [dtheta1; d_d1; dtheta11; dtheta21; dtheta31]
            %   Outputs:
            %       J_a_tool, J_a_P: actuation body jacobian of tool frame and P frame
            %       J_a_pkm: actuation jacobian in pkm frame

            %some basic utils
            I_tilt = [0, -1; 1, 0];

            Tf_P_0 = getInvSE3(obj.Tf_P);
            Tf_P_P0 = Tf_P_0 * obj.Tf_0_P0;

            p_yz = obj.Tf_P(2:3, 4);
            r_sq = p_yz' * p_yz;

            rotm_pkm_P = obj.Tf_pkm_P(1:3, 1:3);

            rotm_P = obj.Tf_P(1:3, 1:3);

            twist_vx = [0, 0, 0, 1, 0, 0];
            twist_vy = [0, 0, 0, 0, 1, 0];
            twist_vz = [0, 0, 0, 0, 0, 1];

            %local jacobian
            J_a_pkm = obj.RRS_2RRU.getActuationJacob();
            obj.RRS_2RRU.getPassiveJacob(); %update passive jacobian part
            
            %P-Point body twist actuation jacobian(in body frame)
            J_a_P_1 = - 1/r_sq * p_yz' * I_tilt * [twist_vy; twist_vz] * blkdiag(rotm_P, rotm_P);
            
            J_a_P_2 = twist_vx * blkdiag(rotm_P, rotm_P) * obj.Ad_C3_P;
            
            obj.J_Opkm_P = adjointMatrix(Tf_P_P0) * (adjointMatrix(expMapSE3(obj.twist_2, -obj.d1)) * obj.twist_1 * J_a_P_1 + obj.twist_2 * J_a_P_2);
            obj.J_pkmP_P = eye(6) - obj.J_Opkm_P;
            J_a_P_3 = J_a_pkm *  blkdiag(rotm_pkm_P, rotm_pkm_P) * obj.J_pkmP_P;

            J_a_P = [J_a_P_1; J_a_P_2; J_a_P_3];

            %tool tip body twist actuation jacobian(in body frame)
            J_a_tool = J_a_P * obj.Ad_P_tool;

            obj.J_a_tool = J_a_tool;
            obj.J_a_P = J_a_P;
        end
        
        function [dJ_a_tool, dJ_a_P] = getActuationBodyJacobDiff(obj, dX_tool)
            %GETACTUATIONBODYJACOBDIFF get actuation body jacobian derivative from tool tip/P-Point body twist to generalized coordinates
            dX_P = obj.Ad_P_tool * dX_tool;
            
            if isempty(obj.J_a_P)
                obj.getActuationBodyJacob();
            end

            e4 = [0, 0, 0, 1, 0, 0];
            e5 = [0, 0, 0, 0, 1, 0];
            e6 = [0, 0, 0, 0, 0, 1];

            %dJ_a for theta1
            I_tilt = [0, -1; 1, 0];
            rotm_P = obj.Tf_P(1:3, 1:3);
            
            p_yz = obj.Tf_P(2:3, 4);
            dp = rotm_P * dX_P(4:6);
            dp_yz = dp(2:3);
            r_sq = p_yz' * p_yz;
            
            omega_P_vedge = skewMatrix(dX_P(1:3));
            temp = I_tilt * [e5; e6] * blkdiag(rotm_P, rotm_P);
            dJ_a_P1 = -1/r_sq * (dp_yz' * temp + p_yz' * temp * blkdiag(omega_P_vedge, omega_P_vedge) + 2*dp_yz'*p_yz*obj.J_a_P(1,:));

            %dJ_a for d1
            dJ_a_P2 = e4 * blkdiag(rotm_P, rotm_P) * blkdiag(omega_P_vedge, omega_P_vedge) * obj.Ad_C3_P;
            
            %dJ_a for thetas in pkm
            dX_pkm_P = obj.J_pkmP_P * dX_P;
            ad_dX_P0_P = adjointTwist(dX_pkm_P);

            Ad_P_P0 = adjointMatrix(getInvSE3(obj.Tf_P) * obj.Tf_0_P0);
            
            d_d1 = obj.J_a_P(2, :) * dX_P;
            ad_dX_Opkm = adjointTwist(obj.twist_2*d_d1);
            Ad_dX_p0O = adjointMatrix(expMapSE3(obj.twist_2, -obj.d1));
            dJ_pkmP_P = ad_dX_P0_P*obj.J_Opkm_P - Ad_P_P0*(-ad_dX_Opkm*Ad_dX_p0O*obj.twist_1*obj.J_a_P(1, :) + ...
                                        Ad_dX_p0O*obj.twist_1*dJ_a_P1 + obj.twist_2*dJ_a_P2);
            
            blk_rotm_pkm_P = blkdiag(obj.Tf_pkm_P(1:3, 1:3), obj.Tf_pkm_P(1:3, 1:3));
            dJ_a_pkm = obj.RRS_2RRU.getActuationJacobDiff(blk_rotm_pkm_P*dX_pkm_P);
            omega_pmk_P_vedge = skewMatrix(dX_pkm_P(1:3));
            dJ_a_P35 = dJ_a_pkm*blk_rotm_pkm_P*obj.J_pkmP_P + obj.RRS_2RRU.J_a*blk_rotm_pkm_P*(blkdiag(omega_pmk_P_vedge, omega_pmk_P_vedge)*obj.J_pkmP_P + dJ_pkmP_P);
            
            %jacobian derivative
            dJ_a_P = [dJ_a_P1; dJ_a_P2; dJ_a_P35];
            dJ_a_tool = dJ_a_P * obj.Ad_P_tool;
        end
        
        %% Tool position constrained inverse kinematics and jacobians w.r.t alpha, beta (2R): 
        
        function config = getEndPosConInvKine2R(obj, P_tool, alpha, beta)
            %GETENDPOSCONINVKINE2R get end-position(tool tip position in base frame) constrained inverse kinematics w.r.t 2R (alpha, beta angles)
            %   Inputs:
            %       P_tool: 3X1

            %get d1
            l_pkm_tool = rotY(beta)*rotX(alpha)*[0; 0; obj.RRS_2RRU.toolHight];
            delta_Px = -sin(alpha)*sin(beta)*obj.RRS_2RRU.r;
            obj.d1 = P_tool(1) - l_pkm_tool(1) - delta_Px;

            %get z_p
            z_p = sqrt(P_tool(2:3)'*P_tool(2:3) - (l_pkm_tool(2)+obj.pkm_y_offset)^2) - (obj.pkm_z_offset+l_pkm_tool(3));

            %get thetas in pkm
            [Tf_BTC_local, ~] = obj.RRS_2RRU.setEndEffectorSE3(z_p, alpha, beta); 
            thetas_pkm = obj.RRS_2RRU.invKineUpdate(Tf_BTC_local);
            
            %get theta1
            P_1 = [obj.d1+delta_Px; obj.pkm_y_offset; z_p+obj.pkm_z_offset];
            obj.theta1 = subIK_rotateOneAxis(obj.twist_1, P_1+l_pkm_tool, P_tool);

            %upate obj.Tf_BTC
            ExpSO3_theta1 = expMapSO3(obj.twist_1(1:3), obj.theta1);
            rotm_P = ExpSO3_theta1 * rotY(beta) * rotX(alpha);
            P = ExpSO3_theta1 * P_1;
            obj.Tf_BTC = [rotm_P, P_tool;
                                      zeros(1,3), 1];
            obj.Tf_P = [rotm_P,   P;
                                  zeros(1,3), 1];

            %update Tf_0_P0, Tf_0_pkm, Tf_pkm_P
            obj.Tf_0_P0 = expMapSE3(obj.twist_1, obj.theta1) * expMapSE3(obj.twist_2, obj.d1);
            obj.Tf_0_pkm = obj.Tf_0_P0 * trvec2tform([0, obj.pkm_y_offset, obj.pkm_z_offset]);
            obj.Tf_pkm_P =  getInvSE3(obj.Tf_0_pkm) * obj.Tf_P;

            %check if empty IK in pkm
            config = [];
            if ~isempty(thetas_pkm)
                config = [obj.theta1, obj.d1, thetas_pkm];
            end
        end

        function [J_a_2R, J_a_Pe, J_a_Pe_b] = getEndPosConJacobian2R(obj)
            %GETENDPOSCONJACOBIAN get end-position(tool tip position) constrained actuation jacobian w.r.t 2R (alpha, beta angles)
            %                                                      dq = J_a_2R*[dalpha; dbeta] + J_a_Pe*dP_tool/dt;
            %   Outputs:
            %       J_a_2R: actuation jacobian w.r.t 2R (alpha, beta angles)
            %       J_a_Pe: actuation jacobian w.r.t dP_tool/dt in base frame
            %       J_a_Pe_b: actuation body jacobian w.r.t dP_tool/dt in BTC frame
            
            s_theta1 = rotX(-obj.RRS_2RRU.alpha)*rotY(-obj.RRS_2RRU.beta)*[1;0;0];
            s_beta = rotX(-obj.RRS_2RRU.alpha)*[0;1;0];
            s_alpha = [1;0;0];

            if isempty(obj.J_a_P)
                obj.getActuationBodyJacob();
            end
            
            k_2R = 1 - obj.J_a_tool(1, 1:3)*s_theta1;
            J_2R_theta1 = obj.J_a_tool(1, 1:3)/k_2R * [s_alpha, s_beta];
            J_Pe_theta1 = obj.J_a_tool(1, 4:6)/k_2R;

            J_a_2R = obj.J_a_tool(:, 1:3)*(s_theta1*J_2R_theta1 + [s_alpha, s_beta]);
            J_a_Pe_b = obj.J_a_tool(:, 1:3)*s_theta1*J_Pe_theta1 + obj.J_a_tool(:, 4:6);
            J_a_Pe = J_a_Pe_b * obj.Tf_P(1:3, 1:3)';
        end

        function [dJ_a_2R, dJ_a_Pe, dJ_a_Pe_b] = getEndPosConJacobian2RDiff(obj, dalpha, dbeta, dP_tool)
            %GETENDPOSCONJACOBIAN2RDIFF get the derivative of end-position(tool tip position) constrained actuation jacobian w.r.t 2R (alpha, beta angles)
            %                                                      ddq = dJ_a_2R*[dalpha; dbeta] + J_a_2R*[ddalpha; ddbeta] + J_a_Pe*ddP_tool/dt + dJ_a_Pe*dP_tool/dt;
            %   Outputs:
            %       dJ_a_2R: the derivative of actuation jacobian w.r.t 2R (alpha, beta angles)
            %       dJ_a_Pe: the derivative of actuation jacobian w.r.t dP_tool/dt in base frames
            %       dJ_a_Pe_b: the derivative of actuation body jacobian w.r.t dP_tool/dt in BTC frames
            
            rot_alpha_inv = rotX(-obj.RRS_2RRU.alpha);
            rot_beta_inv = rotY(-obj.RRS_2RRU.beta);
            s_theta1 = rot_alpha_inv*rot_beta_inv*[1;0;0];
            s_beta = rot_alpha_inv*[0;1;0];
            s_alpha = [1;0;0];
            
            d2R = [dalpha; dbeta];

            if isempty(obj.J_a_P)
                obj.getActuationBodyJacob();
            end
            
            k_2R = 1 - obj.J_a_tool(1, 1:3)*s_theta1;
            J_2R_omega = [s_alpha, s_beta];
            J_2R_theta1 = obj.J_a_tool(1, 1:3)/k_2R * J_2R_omega;
            J_Pe_theta1 = obj.J_a_tool(1, 4:6)/k_2R;
            
            J_a_Pe_b = obj.J_a_tool(:, 1:3)*s_theta1*J_Pe_theta1 + obj.J_a_tool(:, 4:6);

            rotm_P = obj.Tf_P(1:3, 1:3);
            dP_tool_b = rotm_P' * dP_tool;
            dtheta_1 = J_2R_theta1*d2R + J_Pe_theta1*dP_tool_b;

            omega_tool_b = s_theta1*dtheta_1 + s_beta*dbeta + s_alpha*dalpha;
            dX_tool_b = [omega_tool_b; dP_tool_b];
            [dJ_a_tool, ~] = obj.getActuationBodyJacobDiff(dX_tool_b);
            
            ex = [1;0;0];
            ey=[0;1;0];
            ds_theta1_d2R = -[rot_alpha_inv*skewMatrix(ex)*rot_beta_inv*ex, rot_alpha_inv*rot_beta_inv*skewMatrix(ey)*ex];
            ds_theta1 = ds_theta1_d2R*d2R;
            ds_beta = rot_alpha_inv*skewMatrix(ex)*ey*(-dalpha);
            dJ_2R_omega = [zeros(3,1), ds_beta];
            dk_2R = -(dJ_a_tool(1, 1:3)*s_theta1 + obj.J_a_tool(1, 1:3)*ds_theta1);
            dJ_2R_theta1 = (dJ_a_tool(1, 1:3)*J_2R_omega + obj.J_a_tool(1, 1:3)*dJ_2R_omega - dk_2R*J_2R_theta1) / k_2R;
            dJ_Pe_theta1 = (dJ_a_tool(1, 4:6) - dk_2R*J_Pe_theta1) / k_2R;

            dJ_a_2R = dJ_a_tool(:, 1:3)*(s_theta1*J_2R_theta1 + J_2R_omega) + ...
                                obj.J_a_tool(:, 1:3)*(ds_theta1*J_2R_theta1 + s_theta1*dJ_2R_theta1 + dJ_2R_omega);
            dJ_a_Pe_b = dJ_a_tool(:, 1:3)*s_theta1*J_Pe_theta1 + obj.J_a_tool(:, 1:3)*ds_theta1*J_Pe_theta1+ ...
                                obj.J_a_tool(:, 1:3)*s_theta1*dJ_Pe_theta1 + dJ_a_tool(:, 4:6);
            dJ_a_Pe = (dJ_a_Pe_b + J_a_Pe_b*skewMatrix(-omega_tool_b)) * obj.Tf_P(1:3, 1:3)';
        end
    end
end