function J_a2R = Hyd_Ja2R_2(in1,in2)
%Hyd_Ja2R_2
%    J_a2R = Hyd_Ja2R_2(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 23.2.
%    2023-12-24 11:39:23

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
t18 = Pe2.*t4.*1.368e-1;
t19 = Pe3.*t4.*1.368e-1;
t20 = t2.*t5.*(-1.3e+1./1.25e+2);
t21 = t3.*t4.*(-1.3e+1./1.25e+2);
t23 = t2.*t3.*1.368e-1;
t24 = t2.*t5.*1.368e-1;
t25 = t3.*t4.*1.368e-1;
t12 = -t9;
t13 = -t10;
t14 = t4.*t11;
t15 = t3.*t9;
t16 = t5.*t9;
t17 = t4.*t10;
t22 = -t18;
t26 = -t23;
t27 = -t24;
t29 = t9-1.3e+1./1.25e+2;
t30 = t8.*1.871424e-2;
t34 = t11+t21;
t35 = t10+t24;
t31 = -t30;
t32 = t29.^2;
t33 = t11+t17;
t36 = t10+t27;
t39 = t4.*t29.*(2.6e+1./1.25e+2);
t40 = t13+t14+1.3e+1./1.25e+2;
t41 = t10+t14-1.3e+1./1.25e+2;
t37 = t6+t7+t31;
t42 = sign(t40);
t43 = sign(t41);
t44 = abs(t40);
t45 = abs(t41);
t46 = sqrt(t37);
t50 = t44.^2;
t51 = t45.^2;
t102 = t2.*t5.*t42.*t44.*(2.6e+1./1.25e+2);
t103 = t2.*t5.*t43.*t45.*(2.6e+1./1.25e+2);
t109 = t33.*t42.*t44.*2.0;
t110 = t34.*t43.*t45.*2.0;
t47 = conj(t46);
t48 = imag(t46);
t49 = real(t46);
t52 = 1.0./t46;
t53 = -t46;
t80 = t11+t26+t46-2.16e-2;
t90 = t17+t26+t46-2.16e-2;
t54 = Pe2.*t48;
t55 = Pe3.*t48;
t56 = Pe2.*t49;
t57 = Pe3.*t49;
t58 = -t47;
t59 = 1.0./t47;
t60 = -t49;
t61 = t2.*t4.*t52;
t64 = t12+t48+1.3e+1./1.25e+2;
t72 = t40+t48;
t73 = t41+t48;
t81 = t11+t23+t53+2.16e-2;
t82 = abs(t80);
t83 = t11+t26+t47-2.16e-2;
t84 = t11+t26+t49-2.16e-2;
t92 = abs(t90);
t93 = t17+t26+t47-2.16e-2;
t94 = t17+t26+t49-2.16e-2;
t113 = t35.*t80;
t119 = (t24-t4.*t5.*(1.3e+1./1.25e+2)).*(t21+t23+t53+2.16e-2);
t62 = imag(t61);
t63 = real(t61);
t65 = t61.*1.871424e-2;
t67 = t64.^2;
t70 = t2.*t4.*t59.*1.871424e-2;
t74 = t72.^2;
t75 = t73.^2;
t76 = 1.0./t72;
t77 = 1.0./t73;
t85 = t11+t23+t60+2.16e-2;
t88 = abs(t81);
t89 = t11+t23+t58+2.16e-2;
t91 = t84.^2;
t95 = t82.^2;
t99 = t92.^2;
t100 = t94.^2;
t107 = t19+t55+t56;
t108 = t22+t54+t57;
t114 = t35.*t83;
t116 = t36.*t81;
t120 = -t93.*(t24-t4.*t5.*(1.3e+1./1.25e+2));
t133 = t80.*t83;
t141 = -t93.*(t21+t23+t53+2.16e-2);
t66 = -t65;
t68 = t62.*1.871424e-2;
t69 = t63.*1.871424e-2;
t71 = -t70;
t78 = 1.0./t74;
t79 = 1.0./t75;
t96 = t88.^2;
t97 = t85.^2;
t111 = t108.^2;
t115 = t32+t99;
t117 = t36.*t89;
t123 = t51+t95;
t131 = t67+t100;
t134 = t75+t91;
t136 = t81.*t89;
t145 = 1.0./sqrt(t133);
t148 = 1.0./sqrt(t141);
t155 = t113+t114;
t157 = t119+t120;
t86 = t25+t66;
t87 = t20+t68;
t101 = t25+t71;
t118 = 1.0./t115;
t121 = 1.0./sqrt(t115);
t124 = t50+t96;
t125 = 1.0./t123;
t127 = 1.0./sqrt(t123);
t132 = 1.0./t131;
t135 = t74+t97;
t139 = t115-1.903615999999999e-2;
t142 = 1.0./t134;
t147 = 1.0./sqrt(t136);
t149 = t123-1.903615999999999e-2;
t156 = t116+t117;
t173 = t82.*t145.*t155;
t105 = t15+t86;
t106 = t15+t101;
t122 = t121.^3;
t126 = 1.0./t124;
t128 = t127.^3;
t129 = 1.0./sqrt(t124);
t137 = t83.*t86;
t138 = t86.*t89;
t140 = t80.*t101;
t143 = t139.^2;
t144 = 1.0./t135;
t146 = t81.*t101;
t150 = t124-1.903615999999999e-2;
t152 = t149.^2;
t174 = t88.*t147.*t156;
t130 = t129.^3;
t151 = t93.*t105;
t153 = t150.^2;
t154 = -t106.*(t21+t23+t53+2.16e-2);
t158 = t118.*t143.*5.778476331360947;
t162 = t125.*t152.*5.778476331360947;
t168 = t137+t140;
t170 = t138+t146;
t177 = t109+t174;
t159 = -t158;
t163 = -t162;
t164 = t126.*t153.*5.778476331360947;
t172 = t151+t154;
t176 = t82.*t145.*t168;
t178 = t88.*t147.*t170;
t160 = t159+1.0;
t165 = t163+1.0;
t166 = -t164;
t179 = t92.*t148.*t172;
t182 = t103+t176;
t161 = 1.0./sqrt(t160);
t167 = t166+1.0;
t169 = 1.0./sqrt(t165);
t180 = -t179;
t171 = 1.0./sqrt(t167);
t181 = t39+t180;
mt1 = [(t111.*((Pe3.*t2.*(-1.368e-1)+Pe2.*t69+Pe3.*t68)./t108-(t107.*(Pe2.*t2.*1.368e-1+Pe2.*t68+Pe3.*t69))./t111))./(t111+t107.^2),t16+t4.*t5.*1.368e-1,t171.*(t129.*(t102-t178).*(1.25e+2./5.2e+1)-t130.*t150.*(t102-t178).*(1.25e+2./1.04e+2))-t74.*t144.*(t76.*(t25-t69)-t78.*t85.*t87),-t169.*(t127.*t182.*(1.25e+2./5.2e+1)-t128.*t149.*t182.*(1.25e+2./1.04e+2))-t75.*t142.*(t77.*(t25-t69)+t79.*t84.*t87)];
mt2 = [-t161.*(t121.*t181.*(1.25e+2./5.2e+1)-t122.*t139.*t181.*(1.25e+2./1.04e+2))-t67.*t132.*((t15+t25-t69)./t64-(t94.*(t4.*(1.3e+1./1.25e+2)-t68))./t67),0.0,t17+t26,t171.*(t129.*t177.*(1.25e+2./5.2e+1)-t130.*t150.*t177.*(1.25e+2./1.04e+2))+t74.*t144.*(t36.*t76-t33.*t78.*t85),t169.*(t127.*(t110-t173).*(1.25e+2./5.2e+1)-t128.*t149.*(t110-t173).*(1.25e+2./1.04e+2))-t75.*t142.*(t35.*t77+t34.*t79.*t84)];
mt3 = [-t161.*(t92.*t121.*t148.*t157.*(1.25e+2./5.2e+1)-t92.*t122.*t139.*t148.*t157.*(1.25e+2./1.04e+2))-t64.*t132.*(t24-t4.*t5.*(1.3e+1./1.25e+2))];
J_a2R = reshape([mt1,mt2,mt3],5,2);
end