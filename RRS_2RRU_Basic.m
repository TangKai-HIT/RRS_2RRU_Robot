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

    properties(Access=private)
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
    end

    methods
        function obj = RRS_2RRU_Basic(R, r, L1, L2, toolHight)
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
            
            %theta11
            nom = 2*obj.L1*vec_A1C1(3) - sqrt(((obj.L1+obj.L2)^2-vec_A1C1(1)^2-vec_A1C1(3)^2) * (vec_A1C1(1)^2+vec_A1C1(3)^2 - (obj.L1-obj.L2)^2));
            demon = (vec_A1C1(1)+obj.L1)^2 + vec_A1C1(3)^2 - obj.L2^2;
            thetas(1) = 2*atan(nom/demon);
            
            %theta12
            nom = 2*obj.L1*vec_A2C2(3) - sqrt(((obj.L1+obj.L2)^2-vec_A2C2(1)^2-vec_A2C2(3)^2) * (vec_A2C2(1)^2+vec_A2C2(3)^2 - (obj.L1-obj.L2)^2));
            demon = (vec_A2C2(1)+obj.L1)^2 + vec_A2C2(3)^2 - obj.L2^2;
            thetas(2) = pi - 2*atan(nom/demon);

            %theta13
            nom = 2*obj.L1*vec_A3C3(3) - sqrt(((obj.L1+obj.L2)^2-vec_A3C3(2)^2-vec_A3C3(3)^2) * (vec_A3C3(2)^2+vec_A3C3(3)^2 - (obj.L1-obj.L2)^2));
            demon = (vec_A3C3(2)+obj.L1)^2 + vec_A3C3(3)^2 - obj.L2^2;
            thetas(3) = 2*atan(nom/demon);
            
            obj.thetas = thetas;

            %update Bi
            obj.B1 = obj.A1 + eul2rotm([0, -thetas(1), 0], "ZYX") * [obj.L1; 0; 0];
            obj.B2 = obj.A2 + eul2rotm([0, -thetas(2), 0], "ZYX") * [obj.L1; 0; 0];
            obj.B3 = obj.A3 + eul2rotm([0, 0, thetas(3)], "ZYX") * [0; 0; obj.L1];
        end
        
        function J_a = getActuationJacob(obj)
            %GETACTUATIONJACOB J_a*dX_p = [dtheta11; dtheta21; dtheta31]
            b1 = obj.B1 - obj.A1;
            b2 = obj.B2 - obj.A2;
            b3 = obj.B3 - obj.A3;

            c11 = obj.C1 - obj.B1;
            c21 = obj.C2 - obj.B2;
            c31 = obj.C3 - obj.B3;

            c12 = obj.C1 - obj.P;
            c22 = obj.C2 - obj.P;
            c32 = obj.C3 - obj.P;

            s1 = [0;1;0];
            s2 = [0;1;0];
            s3 = [1;0;0];

            J_1a = [obj.r*(cross(c12,c11)'), c11'] ./ (obj.L1*dot(cross(b1,c11), s1));
            J_2a = [obj.r*(cross(c22,c21)'), c21'] ./ (obj.L1*dot(cross(b2,c21), s2));
            J_3a = [obj.r*(cross(c32,c31)'), c31'] ./ (obj.L1*dot(cross(b3,c31), s3));

            J_a = [J_1a; J_2a; J_3a];
        end
        
        function R_p = calEndEffectorSO3(obj, alpha, beta)
            %CALENDEFFECTORSE3
            R_p = [cos(beta),  sin(alpha)*sin(beta),  cos(alpha)*sin(beta);
                    0   ,       cos(alpha),         -sin(alpha);
                -sin(beta), sin(alpha)*cos(beta),  cos(alpha)*cos(beta)];
        end

        function [Tf_BTC, Tf_P] = calEndEffectorSE3(obj, zp, alpha, beta)
            %CALENDEFFECTORSE3
            R_p = obj.calEndEffectorSO3(alpha, beta);

            P_0 = [-sin(alpha)*sin(beta)*obj.r; 0; zp];

            Tf_P = [R_p, P_0;
                    zeros(1,3),  1];

            btc_p = [0, 0, obj.toolHight];
            Tf_BTC = Tf_P * trvec2tform(btc_p);
        end

        function [J_rt, J_rp] = getOutputJacob(obj, alpha, beta)
            %GETOUTPUTJACOB
            J_rp_T = [0, -cos(alpha)*sin(beta)*obj.r, -sin(alpha)*cos(beta)*obj.r;
                      0,           0,                           0;
                      1,            0,                          0];
            
            J_rp_R = [0,    cos(beta),  0;
                      0,      0,        1;
                      0,    -sin(beta), 0];

            J_rp = [J_rp_R; J_rp_T];
            
            R_p = obj.calEndEffectorSO3(alpha, beta);
            r_tool = R_p * [0; 0; obj.toolHight];
            trans = [eye(6),        zeros(6);
                    -skewMatrix(r_tool),     eye(6)];
            
            J_rt = trans * J_rp;
        end

        function   torque_actuate = getStaticForce_tool(obj, torque_body, force_body)
            %GETSTATICFORCE_TOOL get equilibrium torque on actuators when setting wrench on tool center 
            %   Inputs:
            %       torque_body, force_body: 3 X 1, torque & force in local BTC frame

            J_a = obj.getActuationJacob();
            wrench_tool = [torque_body; force_body]; %wrench in tool center frame
            
            %transform (adjoint map to P frame)
            r_tool = obj.R_P * [0; 0; obj.toolHight];
            trans = [eye(6),        zeros(6);
                    -skewMatrix(r_tool),     eye(6)];
            
            wrench_P = trans' * blkdiag(obj.R_P, obj.R_P) * wrench_tool;

            torque_actuate = pinv(J_a') * wrench_P;
        end

    end
end