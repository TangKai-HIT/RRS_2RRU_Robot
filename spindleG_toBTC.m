function wrench = spindleG_toBTC(RRS_2RRU)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

%transform (adjoint map to P frame that aligned with base frame)
btc_spindleC = [0; 0; RRS_2RRU.toolHight] - RRS_2RRU.spindle_mass_center_p;
Tf_ma_BTC = [RRS_2RRU.R_P,    RRS_2RRU.R_P*btc_spindleC;
                                    zeros(1,3),           1];
wrench_G = [zeros(3,1); RRS_2RRU.spindle_mass*RRS_2RRU.gravity];
wrench = (adjointMatrix(Tf_ma_BTC)') * wrench_G;

end