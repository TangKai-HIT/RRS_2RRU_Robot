addpath(genpath(pwd));

[RRS_2RRU, robot_file] = smimport('RRS_2RRU.xml','ModelName','RRS_2RRU_Robot',...
'DataFileName','RRS_2RRU_data_file');