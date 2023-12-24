function J_a_Pe = Hyd_JaPe_2(in1,in2)
%Hyd_JaPe_2
%    J_a_Pe = Hyd_JaPe_2(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 23.2.
%    2023-12-24 11:39:24

Pe2 = in2(2,:);
Pe3 = in2(3,:);
alpha = in1(:,1);
beta = in1(:,2);
t2 = cos(alpha);
t3 = cos(beta);
t4 = sin(alpha);
t5 = sin(beta);
t6 = Pe2.^2;
t7 = Pe3.^2;
t8 = t4.^2;
t9 = t2.*(1.3e+1./1.25e+2);
t10 = t3.*(1.3e+1./1.25e+2);
t11 = t5.*(1.3e+1./1.25e+2);
t15 = t4.*1.368e-1;
t19 = Pe2.*t4.*(-1.368e-1);
t20 = t2.*t3.*1.368e-1;
t12 = -t9;
t13 = -t10;
t14 = t4.*t11;
t16 = t4.*t10;
t18 = Pe3.*t15;
t21 = -t20;
t22 = t9-1.3e+1./1.25e+2;
t23 = t8.*1.871424e-2;
t24 = -t23;
t25 = t22.^2;
t27 = t13+t14+1.3e+1./1.25e+2;
t28 = t10+t14-1.3e+1./1.25e+2;
t26 = t6+t7+t24;
t29 = abs(t27);
t30 = abs(t28);
t31 = sqrt(t26);
t35 = t29.^2;
t36 = t30.^2;
t32 = conj(t31);
t33 = imag(t31);
t34 = real(t31);
t37 = 1.0./t31;
t38 = -t31;
t64 = t11+t21+t31-2.16e-2;
t72 = t16+t21+t31-2.16e-2;
t39 = Pe2.*t37;
t40 = Pe3.*t37;
t41 = Pe2.*t33;
t42 = Pe3.*t33;
t43 = Pe2.*t34;
t44 = Pe3.*t34;
t45 = -t32;
t46 = 1.0./t32;
t47 = -t34;
t52 = t12+t33+1.3e+1./1.25e+2;
t56 = t27+t33;
t57 = t28+t33;
t65 = t11+t20+t38+2.16e-2;
t66 = abs(t64);
t67 = t11+t21+t32-2.16e-2;
t68 = t11+t21+t34-2.16e-2;
t74 = abs(t72);
t75 = t16+t21+t32-2.16e-2;
t76 = t16+t21+t34-2.16e-2;
t48 = imag(t39);
t49 = imag(t40);
t50 = real(t39);
t51 = real(t40);
t53 = t52.^2;
t54 = 1.0./t52;
t58 = t56.^2;
t59 = t57.^2;
t60 = 1.0./t56;
t61 = 1.0./t57;
t69 = t11+t20+t47+2.16e-2;
t70 = abs(t65);
t71 = t11+t20+t45+2.16e-2;
t73 = t68.^2;
t77 = t66.^2;
t80 = t74.^2;
t81 = t76.^2;
t82 = t18+t42+t43;
t83 = t19+t41+t44;
t92 = t39.*t67;
t93 = t40.*t67;
t98 = Pe2.*t46.*t64;
t99 = Pe3.*t46.*t64;
t100 = t39.*t75;
t101 = t40.*t75;
t102 = Pe2.*t46.*t65;
t103 = Pe3.*t46.*t65;
t105 = -Pe2.*t46.*(t20+t38-t3.*t4.*(1.3e+1./1.25e+2)+2.16e-2);
t106 = -Pe3.*t46.*(t20+t38-t3.*t4.*(1.3e+1./1.25e+2)+2.16e-2);
t114 = t64.*t67;
t119 = -t75.*(t20+t38-t3.*t4.*(1.3e+1./1.25e+2)+2.16e-2);
t55 = 1.0./t53;
t62 = 1.0./t58;
t63 = 1.0./t59;
t78 = t70.^2;
t79 = t69.^2;
t84 = t82.^2;
t85 = t83.^2;
t86 = 1.0./t83;
t88 = t25+t80;
t94 = t39.*t71;
t95 = t40.*t71;
t96 = t36+t77;
t112 = t53+t81;
t115 = t59+t73;
t117 = t65.*t71;
t123 = 1.0./sqrt(t114);
t125 = 1.0./sqrt(t119);
t132 = t92+t98;
t133 = t93+t99;
t136 = t100+t105;
t137 = t101+t106;
t87 = 1.0./t85;
t89 = 1.0./t88;
t90 = 1.0./sqrt(t88);
t97 = t35+t78;
t104 = 1.0./t96;
t108 = 1.0./sqrt(t96);
t113 = 1.0./t112;
t116 = t58+t79;
t118 = t88-1.903615999999999e-2;
t120 = 1.0./t115;
t124 = 1.0./sqrt(t117);
t126 = t96-1.903615999999999e-2;
t130 = t84+t85;
t134 = t94+t102;
t135 = t95+t103;
t91 = t90.^3;
t107 = 1.0./t97;
t109 = t108.^3;
t110 = 1.0./sqrt(t97);
t121 = t118.^2;
t122 = 1.0./t116;
t127 = t97-1.903615999999999e-2;
t128 = t126.^2;
t131 = 1.0./t130;
t111 = t110.^3;
t129 = t127.^2;
t138 = t89.*t121.*5.778476331360947;
t142 = t104.*t128.*5.778476331360947;
t139 = -t138;
t143 = -t142;
t144 = t107.*t129.*5.778476331360947;
t140 = t139+1.0;
t145 = t143+1.0;
t146 = -t144;
t141 = 1.0./sqrt(t140);
t147 = t146+1.0;
t148 = 1.0./sqrt(t145);
t149 = 1.0./sqrt(t147);
mt1 = [0.0,1.0,0.0,0.0,0.0,-t85.*t131.*(t86.*(t34+Pe3.*t48+Pe2.*t50)-t82.*t87.*(-t15+t33+Pe2.*t48+Pe3.*t50)),0.0,-t149.*(t70.*t110.*t124.*t134.*(1.25e+2./5.2e+1)-t70.*t111.*t124.*t127.*t134.*(1.25e+2./1.04e+2))-t58.*t122.*(t50.*t60+t48.*t62.*t69),-t148.*(t66.*t108.*t123.*t132.*(1.25e+2./5.2e+1)-t66.*t109.*t123.*t126.*t132.*(1.25e+2./1.04e+2))-t59.*t120.*(t50.*t61-t48.*t63.*t68),t141.*(t74.*t90.*t125.*t136.*(1.25e+2./5.2e+1)-t74.*t91.*t118.*t125.*t136.*(1.25e+2./1.04e+2))-t53.*t113.*(t50.*t54-t48.*t55.*t76)];
mt2 = [-t85.*t131.*(t86.*(t15+t33+Pe3.*t49+Pe2.*t51)-t82.*t87.*(t34+Pe2.*t49+Pe3.*t51)),0.0,-t149.*(t70.*t110.*t124.*t135.*(1.25e+2./5.2e+1)-t70.*t111.*t124.*t127.*t135.*(1.25e+2./1.04e+2))-t58.*t122.*(t51.*t60+t49.*t62.*t69),-t148.*(t66.*t108.*t123.*t133.*(1.25e+2./5.2e+1)-t66.*t109.*t123.*t126.*t133.*(1.25e+2./1.04e+2))-t59.*t120.*(t51.*t61-t49.*t63.*t68),t141.*(t74.*t90.*t125.*t137.*(1.25e+2./5.2e+1)-t74.*t91.*t118.*t125.*t137.*(1.25e+2./1.04e+2))-t53.*t113.*(t51.*t54-t49.*t55.*t76)];
J_a_Pe = reshape([mt1,mt2],5,3);
end