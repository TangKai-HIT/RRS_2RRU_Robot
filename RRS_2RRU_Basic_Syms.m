classdef RRS_2RRU_Basic_Syms < handle
    %RRS_2RRU_BASIC_SYMS Support symbolic computations
    %   Detailed explanation goes here

    properties
        R;
        r;
        L1;
        L2;
        toolHight; %cutter BTC hight (z) in P frame
        gravity = [0;0;-9.8]; %gravity

        %singularity check using condition number
        maxKappa_fkine = 10;
        maxKappa_invkine = 40;

        %max ball joint angle
        maxBallJointAngle = pi/6; %30 degree
    end

    properties(SetAccess=private, GetAccess=public)
        %spindle mass
        spindle_mass;
        spindle_mass_center_p;
        
        %config
        thetas;
        
        %screws
        s11 = [0;-1;0]; %theta11
        s12 = [0;-1;0]; %theta12
        s21 = [0;-1;0]; %theta21
        s22 = [0;-1;0]; %theta22
        s31 = [1;0;0]; %theta31
        s32 = [1;0;0]; %theta32

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
        
        %ball joint installation base normal vector in P frame
        s_n_P;
        s_install_angle = pi/4;

        %jacobians
        J_a; %actuation jacobian
        J_pa; %passive joints (middle R joints) jacobian
    end

    methods
        function obj = RRS_2RRU_Basic_Syms(R, r, L1, L2, toolHight, z_p_min)
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
                obj.s_n_P = rotX(-(pi/2-obj.s_install_angle)) * [0; -1; 0];
            end
        end

        function thetas = invKineUpdate(obj, Tf_BTC)
            %INVKINEUPDATE inverse kinematic of the robot given SE3 of P(end-effector)
            %   Inputs:
            %       Tf_BTC: SE3 of BTC w.r.t base O
            
            btc_p = [0, 0, obj.toolHight];
            Tf_p_btc = mytrvec2tform(btc_p);
            
            obj.TC = Tf_BTC(1:3, 4);
            obj.R_P = Tf_BTC(1:3, 1:3);

            Tf_P = Tf_BTC * getInvSE3(Tf_p_btc);
            obj.P = Tf_P(1:3, 4);
            
            %update 2R-1T
            obj.z_p = obj.P(3);
            r13 = obj.R_P(1,3); r33 = obj.R_P(3,3); r23 = obj.R_P(2,3);
            obj.alpha = atan2(-r23, sqrt(r13^2 + r33^2));
            obj.beta = atan2(r13, r33); %cos(alpha) ~= 0

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
            thetas1 = delta - gamma;
            
            %theta12(R-R-R crank)
            delta = atan2(vec_A2C2(3), vec_A2C2(1));
            d = norm(vec_A2C2);
            gamma = acos((d^2 + obj.L1^2 - obj.L2^2)/(2*d*obj.L1));
            thetas2 = gamma + delta;

            %theta13(R-R-R crank)
            delta = atan2(vec_A3C3(3), vec_A3C3(2));
            d = norm(vec_A3C3);
            gamma = acos((d^2 + obj.L1^2 - obj.L2^2)/(2*d*obj.L1));
            thetas3 = delta - gamma;
            
            thetas = [thetas1, thetas2, thetas3];

            %record result
            obj.thetas = thetas;

            %update Bi
            obj.B1 = obj.A1 + rotY(-thetas(1)) * [obj.L1; 0; 0];
            obj.B2 = obj.A2 + rotY(-thetas(2)) * [obj.L1; 0; 0];
            obj.B3 = obj.A3 + rotX(thetas(3)) * [0; obj.L1; 0];

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
            obj.J_a = J_a;
        end

        function J_pa = getPassiveJacob(obj)
            %GETPASSIVEJACOB J_pa*dX_p = [dtheta12; dtheta22; dtheta32]

            J_pa_1 = [obj.r*cross(obj.c12, obj.b1)', obj.b1']./(obj.L2*dot(cross(obj.c11, obj.b1), obj.s12));
            J_pa_2 = [obj.r*cross(obj.c22, obj.b2)', obj.b2']./(obj.L2*dot(cross(obj.c21, obj.b2), obj.s22));
            J_pa_3 = [obj.r*cross(obj.c32, obj.b3)', obj.b3']./(obj.L2*dot(cross(obj.c31, obj.b3), obj.s32));
            J_pa = [J_pa_1; J_pa_2; J_pa_3];

            obj.J_pa = J_pa;
        end

        function [J_theta_a, J_X_a] = getSplitedActuationJacob(obj)
            %GETSPLITEDACTUATIONJACOB J_theta_a * dtheta = J_X_a * dX_p

            J_theta_a = obj.L1 * blkdiag(dot(cross(obj.b1,obj.c11), obj.s11), dot(cross(obj.b2,obj.c21), obj.s21), dot(cross(obj.b3,obj.c31), obj.s31));
            
            J_X_1a = [obj.r*(cross(obj.c12,obj.c11)'), obj.c11'];
            J_X_2a = [obj.r*(cross(obj.c22,obj.c21)'), obj.c21'];
            J_X_3a = [obj.r*(cross(obj.c32,obj.c31)'), obj.c31'];
            
            J_X_a = [J_X_1a; J_X_2a; J_X_3a];
        end
        
        function dJ_a = getActuationJacobDiff(obj, dX_p)
            %GETACTUATIONJACOBDIFF return dJ_a, ddtheta = J_a*ddX_p + dJ_a*dX_p, must update J_a, J_pa before calling the function 
            %   Inputs:
            %       dX_p
            
            if isempty(obj.J_a) %must update actuation J_a before the function 
                obj.getActuationJacob();
            end

            if isempty(obj.J_pa) %must update passive J_pa before the function 
                obj.getPassiveJacob();
            end

            E = [eye(3), zeros(3,3)];
            dtheta1 = obj.J_a*dX_p;
            dtheta2 = obj.J_pa*dX_p;
            omega_p = dX_p(1:3);
            
            nom = obj.L1*dot(cross(obj.b1,obj.c11), obj.s11);
            dJ_a1 = (obj.r*((obj.c11'*omega_p)*obj.c12' - (obj.c11'*obj.c12)*omega_p')*E +...
                        obj.L1*obj.c11'*obj.b1*dtheta1(1)*obj.J_a(1,:) + obj.L2*dtheta2(1)*obj.J_pa(1,:))./nom;

            nom = obj.L1*dot(cross(obj.b2,obj.c21), obj.s21);
            dJ_a2 = (obj.r*((obj.c21'*omega_p)*obj.c22' - (obj.c21'*obj.c22)*omega_p')*E +...
                        obj.L1*obj.c21'*obj.b2*dtheta1(2)*obj.J_a(2,:) + obj.L2*dtheta2(2)*obj.J_pa(2,:))./nom;

            nom = obj.L1*dot(cross(obj.b3,obj.c31), obj.s31);
            dJ_a3 = (obj.r*((obj.c31'*omega_p)*obj.c32' - (obj.c31'*obj.c32)*omega_p')*E +...
                        obj.L1*obj.c31'*obj.b3*dtheta1(3)*obj.J_a(3,:) + obj.L2*dtheta2(3)*obj.J_pa(3,:))./nom;                      

            dJ_a = [dJ_a1; dJ_a2; dJ_a3];
        end

        function [Kappa_fkine, Kappa_invkine] = manipulationCondNum(obj)
            %MANIPULATIONCONDNUM compute manipulation condition number
            [J_theta_a, J_X_a] = obj.getSplitedActuationJacob();
            [~, J_rp] = obj.getOutputJacob(obj.alpha, obj.beta);
            J_inv = J_X_a * J_rp;

            %forward manipulation Condition Number
            Kappa_fkine = norm(J_theta_a, "fro") * norm(inv(J_theta_a), "fro");
            %inverse manipulation Condition Number
            Kappa_invkine = norm(J_inv, "fro") * norm(inv(J_inv), "fro");
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
        
        function [forwardSingular, inverseSingular] = checkSingularityCondNum(obj)
            %CHECKSINGULARITYCONDNUM check inverse and forward singularity using conditional number of Frobenius norm
            forwardSingular = false;
            inverseSingular = false;
            
            [Kappa_fkine, Kappa_invkine] = obj.manipulationCondNum();

            %check forward singularity
            if Kappa_fkine > obj.maxKappa_fkine
                forwardSingular = true;
            end
            
            %check inverse singularity
            if Kappa_invkine > obj.maxKappa_invkine
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
            Tf_BTC = Tf_P * mytrvec2tform(btc_p);
        end

        function [J_rt, J_rp] = getOutputJacob(obj, alpha, beta)
            %GETOUTPUTJACOB J_rp*[dz_p; dalpha, dbeta] = dX_p
            %   Outputs:
            %       J_rt:OutputJacob to tool tip body twist in base frame
            %       J_rp:OutputJacob to P point body twist in base frame
            
            if nargin<2
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

        function dJ_rp = getOutputJacobDiff(obj, dalpha, dbeta)
            %GETOUTPUTJACOBDIFF  ddX_p = J_rp*[ddz_p; ddalpha; ddbeta] + dJ_rp*[dz_p; dalpha; dbeta]
            %   Inputs:
            %       dalpha, dbeta 
            %   Outputs:
            %       dJ_rp:OutputJacob to P point body twist in base frame 

            dJ_rp_T = [0,  obj.r*(sin(obj.alpha)*sin(obj.beta)*dalpha - cos(obj.alpha)*cos(obj.beta)*dbeta),   obj.r*(-cos(obj.alpha)*cos(obj.beta)*dalpha + sin(obj.alpha)*sin(obj.beta)*dbeta);
                                0,                             0,                           0;
                                0,                             0,                          0];
            
            dJ_rp_R = [0,    -sin(obj.beta)*dbeta,  0;
                              0,      0,        0;
                              0,    -cos(obj.beta)*dbeta, 0];

            dJ_rp = [dJ_rp_R; dJ_rp_T];
        end

        function torque_actuate = getStaticForce_tool(obj, torque_body, force_body)
            %GETSTATICFORCE_TOOL get equilibrium actuation torque on actuators when setting wrench on tool center 
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

        function [dthetas1, dthetas2] = getRRJointsVel(obj, dX_p)
            %GETRRJOINTSVEL get velocity of RR-joints
            %   Output:
            %       dthetas1: 3 X 1, actuation joints
            %       dthetas2: 3 X 1, passive joints

            J_a = getActuationJacob(obj);
            dthetas1 = J_a * dX_p;
            
            J_pa = getPassiveJacob(obj);
            dthetas2 = J_pa * dX_p;
        end

        function ddthetas = getActuationAccel(obj, dX_p, ddX_p)
            %GETACTUATIONACCEL get dotdot_theta under current config
            omega_p = dX_p(1:3);
            
            [dthetas1, dthetas2] = getRRJointsVel(obj, dX_p);
            omega_11 = dthetas1(1)*obj.s11;
            omega_21 = dthetas1(2)*obj.s21;
            omega_31 = dthetas1(3)*obj.s31;
            omega_12 = dthetas2(1)*obj.s12;
            omega_22 = dthetas2(2)*obj.s22;
            omega_32 = dthetas2(3)*obj.s32;
            
            [J_theta_a, J_X_a] = getSplitedActuationJacob(obj);

            delta_11 = obj.r*((obj.c11'*omega_p)*(obj.c12'*omega_p) - (omega_p'*omega_p)*(obj.c11'*obj.c12))...
                        + obj.L1*(omega_11'*omega_11)*(obj.c11'*obj.b1) + obj.L2*(omega_12'*omega_12);
            delta_21 = obj.r*((obj.c21'*omega_p)*(obj.c22'*omega_p) - (omega_p'*omega_p)*(obj.c21'*obj.c22))...
                        + obj.L1*(omega_21'*omega_21)*(obj.c21'*obj.b2) + obj.L2*(omega_22'*omega_22);
            delta_31 = obj.r*((obj.c31'*omega_p)*(obj.c32'*omega_p) - (omega_p'*omega_p)*(obj.c31'*obj.c32))...
                        + obj.L1*(omega_31'*omega_31)*(obj.c31'*obj.b3) + obj.L2*(omega_32'*omega_32);
            delta_1 = [delta_11; delta_21; delta_31];

            ddthetas = (J_theta_a\J_X_a) * ddX_p + (J_theta_a\delta_1);
        end

        function [f, zp_space, maxId] = plotToolWorkSpace(obj, zp_range, alpha_range, beta_range, N_zp)
            %PLOTTOOLWORKSPACE plot tool tip workspace
            %   Outputs:
            %       f: figure object handle
            %       zp_space: discretized z_p slices
            %       maxId: index of the z_p slice having the maximum area of workspace

            N = 100;
            zp_space = linspace(zp_range(1), zp_range(2), N_zp);
            alpha_space = linspace(alpha_range(1), alpha_range(2), N);
            beta_space = linspace(beta_range(1), beta_range(2), N);
                     
            f = figure();
            
            singular_tol = 0.02; %singularity check tolerance
            cond_tol = 0.02; %condition number check tolerance
            
            maxNumSamples = 0; 
            for i = 1:N_zp
                cur_zp = zp_space(i);
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
                    if length(x_tool) > maxNumSamples
                        maxNumSamples = length(x_tool);
                        maxId = i;
                    end

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
        
        function setSpindleMass(obj, mass, mass_center_p)
            %SETSPINDLEMASS
            obj.spindle_mass = mass;
            obj.spindle_mass_center_p = mass_center_p;
        end
        
        function G_actuate = getEquGravityForce(obj)
            %GETEQUGRAVITYFORCE get equilibrium actuation torque on actuators to balance gravity
            G_spindle = obj.spindle_mass * obj.gravity;
            %transform (adjoint map to P frame that aligned with base frame)
            Tf_Pa_spindle = [eye(3),    obj.R_P*obj.spindle_mass_center_p;
                                    zeros(1,3),           1];
            wrench_P = (adjointMatrix(getInvSE3(Tf_Pa_spindle))') * [zeros(3,1); G_spindle];

            J_a = obj.getActuationJacob();
            [~, J_rp] = obj.getOutputJacob();

            G_actuate = - (J_a*J_rp)' \ (J_rp' *  wrench_P);
        end
        
        function [isValid, curAngle] = checkBallJointAngle(obj)
            %CHECKBALLJOINTANGLE check ball joint installation angle if is in the valid range (update IK first)
            s_n = obj.R_P * obj.s_n_P;
            curAngle = acos(dot(s_n, obj.c31));

            isValid = curAngle <= obj.maxBallJointAngle;
        end

    end
end