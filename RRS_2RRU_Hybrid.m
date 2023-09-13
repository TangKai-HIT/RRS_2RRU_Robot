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
        Tf_BTC; %P in base coordinate
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
            end
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
            theta1 = subIK_rotateOneAxis(obj.twist_1, P0, P);
            obj.theta1 = theta1;

            %solve subproblem for alpha, beta
            rotm_P = Tf_BTC(1:3, 1:3);
            rotm_P_local = expMapSO3(obj.twist_1(1:3), -theta1) * rotm_P;

            r13 = rotm_P_local(1,3); r33 = rotm_P_local(3,3); r23 = rotm_P_local(2,3);
            alpha = atan2(-r23, sqrt(r13^2 + r33^2));
            beta = atan2(r13, r33); %cos(alpha) ~= 0

            p_x_local = -sin(alpha)*sin(beta)*obj.RRS_2RRU.r;
            d1 = P(1) - p_x_local;
            obj.d1 = d1;

            %solve invkine of PKM RRS_2RRU
            [Tf_BTC_local, ~] = obj.RRS_2RRU.setEndEffectorSE3(z_p, alpha, beta); 
            thetas_pkm = obj.RRS_2RRU.invKineUpdate(Tf_BTC_local);

            config = [];
            if ~isempty(thetas_pkm)
                config = [theta1, d1, thetas_pkm];
            end
        end

        function [J_a_tool, J_a_P] = getActuationBodyJacob(obj)
            %GETACTUATIONBODYJACOB get actuation body jacobian from tool tip/P-Point body twist to generalized coordinates, 
            %                   J_a_tool*dX_b_tool = J_a_P*dX_b_P = [dtheta1; d_d1; dtheta11; dtheta21; dtheta31]
            
            %some basic utils
            I_tilt = [0, -1; 1, 0];
            Tf_0_P0 = expMapSE3(obj.twist_1, obj.theta1) * expMapSE3(obj.twist_2, obj.d1);
            Tf_0_pkm = Tf_0_P0 * trvec2tform([0, obj.pkm_y_offset, obj.pkm_z_offset]);

            Tf_P_0 = getInvSE3(obj.Tf_P);
            Tf_P_P0 = Tf_P_0 * Tf_0_P0;

            p_yz = obj.Tf_P(2:3, 4);
            r_sq = p_yz' * p_yz;

            Tf_pkm_P =  getInvSE3(Tf_0_pkm) * obj.Tf_P;
            rotm_pkm_P = Tf_pkm_P(1:3, 1:3);

            rotm_P = obj.Tf_P(1:3, 1:3);
            Tf_C3_P = trvec2tform([0,-obj.RRS_2RRU.r, 0]);

            twist_vx = [0, 0, 0, 1, 0, 0];
            twist_vy = [0, 0, 0, 0, 1, 0];
            twist_vz = [0, 0, 0, 0, 0, 1];

            %local jacobian
            J_a_pkm = obj.RRS_2RRU.getActuationJacob();
            
            %P-Point body twist actuation jacobian(in body frame)
            J_a_P_1 = - 1/r_sq * p_yz' * I_tilt * [twist_vy; twist_vz] * blkdiag(rotm_P, rotm_P);
            
            J_a_P_2 = twist_vx * blkdiag(rotm_P, rotm_P) * adjointMatrix(Tf_C3_P);
            
            temp = adjointMatrix(Tf_P_P0) * (adjointMatrix(expMapSE3(obj.twist_2, -obj.d1)) * obj.twist_1 * J_a_P_1 + obj.twist_2 * J_a_P_2);
            J_a_P_3 = J_a_pkm *  blkdiag(rotm_pkm_P, rotm_pkm_P) * (eye(6) - temp);

            J_a_P = [J_a_P_1; J_a_P_2; J_a_P_3];

            %tool tip body twist actuation jacobian(in body frame)
            Tf_P_tool = Tf_P_0 * obj.Tf_BTC;
            J_a_tool = J_a_P * adjointMatrix(Tf_P_tool);
        end
        
        function Tf_tool = forwardKine(obj, theta1, d1, z_p, alpha, beta)
            Tf_pkm = expMapSE3(obj.twist_1, theta1) * expMapSE3(obj.twist_2, d1) * trvec2tform([0, obj.pkm_y_offset, obj.pkm_z_offset]);
            [Tf_pkm_tool, ~] = obj.RRS_2RRU.setEndEffectorSE3(z_p, alpha, beta);
            Tf_tool = Tf_pkm*Tf_pkm_tool;
        end
    end
end