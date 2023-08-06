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
            end
        end

        function [theta1, d1, thetas_pkm] = invKine(obj,Tf_BTC)
            %INVKINE inverse kinematic of hybrid robot
            %   Detailed explanation goes here

            %P pose & position in base coordinate
            Tf_BTC_P = trvec2tform([0 0 -obj.RRS_2RRU.toolHight]);
            Tf_P = Tf_BTC * Tf_BTC_P;
            P = Tf_P(1:3, 4);
            
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
        end
    end
end