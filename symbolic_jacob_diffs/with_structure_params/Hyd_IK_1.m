function configs = Hyd_IK_1(in1,in2,R,r,L1,L2,toolHight,pkm_y_offset,pkm_z_offset)
%Hyd_IK_1
%    CONFIGS = Hyd_IK_1(IN1,IN2,R,r,L1,L2,toolHight,PKM_Y_OFFSET,PKM_Z_OFFSET)

%    This function was generated by the Symbolic Math Toolbox version 23.2.
%    2023-12-24 13:33:14

Pe1 = in2(1,:);
Pe2 = in2(2,:);
Pe3 = in2(3,:);
alpha = in1(:,1);
beta = in1(:,2);
t2 = cos(alpha);
t3 = cos(beta);
t4 = sin(alpha);
t5 = sin(beta);
t6 = L1.^2;
t7 = L2.^2;
t8 = Pe2.^2;
t9 = Pe3.^2;
t14 = 1.0./L1;
t15 = -R;
t16 = -pkm_z_offset;
t17 = pkm_z_offset.*1i;
t10 = r.*t2;
t11 = r.*t3;
t12 = r.*t5;
t13 = t4.*toolHight;
t18 = -t7;
t19 = t2.*t3.*toolHight;
t25 = -t17;
t20 = t4.*t11;
t21 = t4.*t12;
t22 = -t10;
t23 = -t11;
t24 = -t13;
t26 = t12.*1i;
t28 = -t19;
t31 = t19.*1i;
t27 = R+t22;
t29 = pkm_y_offset+t24;
t30 = -t21;
t33 = -t31;
t37 = R+t21+t23;
t38 = t11+t15+t21;
t32 = abs(t27);
t34 = t29.^2;
t39 = abs(t37);
t40 = abs(t38);
t35 = t32.^2;
t36 = -t34;
t41 = t39.^2;
t42 = t40.^2;
t43 = t8+t9+t36;
t44 = sqrt(t43);
t45 = -t44;
t46 = t44.*1i;
t49 = t12+t16+t28+t44;
t52 = t16+t20+t28+t44;
t47 = pkm_z_offset+t12+t19+t45;
t51 = abs(t49);
t54 = abs(t52);
t48 = abs(t47);
t53 = t51.^2;
t55 = t54.^2;
t50 = t48.^2;
configs = [atan2(Pe3.*t29-Pe2.*t44,Pe2.*t29+Pe3.*conj(t44)),Pe1+t21-t2.*t5.*toolHight,-acos((t14.*1.0./sqrt(t41+t50).*(t6+t18+t41+t50))./2.0)+angle(t11+t15+t25-t26+t30+t33+t46),acos((t14.*1.0./sqrt(t42+t53).*(t6+t18+t42+t53))./2.0)+angle(R+t23+t25+t26+t30+t33+t46),-acos((t14.*1.0./sqrt(t35+t55).*(t6+t18+t35+t55))./2.0)+atan2(t20,t10+t15+t25+t33+t46)];
end