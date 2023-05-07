function F_c = roughMillingForce(feedRate, Nt, a_w, a_p, d_0, omega)
%ROUGHMILLINGFORCE rough Milling Force of falt-end milling aluminum alloy

a_f = feedRate*60/(omega*Nt);      % feed per tooth, mm/tooth
C_F = 167; %aluminum alloy
K_F = 1;

%general milling force
F_c = C_F * K_F * a_p^0.86 * a_f^0.72 * d_0^(-0.86) *  Nt * a_w;