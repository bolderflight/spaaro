% Configures and trims simulation
%
% Brian R Taylor
% brian.taylor@bolderflight.com
% 
% Copyright (c) 2022 Bolder Flight Systems
%

%% Autocode setup
run('./autocode_setup');

%% Setup aircraft
run(strcat('./aircraft/', vehicle));

%% Trim
trim();

%% Cleanup
clear vehicle op_point op_report op_spec opt;
