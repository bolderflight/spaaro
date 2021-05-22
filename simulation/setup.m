% Configures and trims simulation
%
% Brian R Taylor
% brian.taylor@bolderflight.com
% 
% Copyright (c) 2021 Bolder Flight Systems
%

%% Cleanup
bdclose all;
close all;
clear all;
clc;

%% Configure
config();

%% Add paths
addpath(genpath('aircraft'));
addpath(genpath('matlab'));
addpath(genpath('models'));
addpath(genpath('control'));

%% Specify root folders for autocode and cache
Simulink.fileGenControl('set', ...
    'CacheFolder', '../flight_code/build/slprj', ...
    'CodeGenFolder', '../flight_code/autocode', ...
    'CodeGenFolderStructure', ...
    Simulink.filegen.CodeGenFolderStructure.ModelSpecific, ...
    'createDir', true);

%% Load bus definitions
load('bus_defs.mat');

%% Call the setup scripts
fh_vehicle = str2func(vehicle);
fh_vehicle();

%% Trim
trim();

%% Create flight plan, fence, and rally point structs
Telem.NUM_FLIGHT_PLAN_POINTS = 100;
Telem.NUM_FENCE_POINTS = 50;
Telem.NUM_RALLY_POINTS = 10;
% Flight plan
for i = 1:Telem.NUM_FLIGHT_PLAN_POINTS
    Telem.FlightPlan(i).autocontinue = boolean(0);
    Telem.FlightPlan(i).frame = uint8(0);
    Telem.FlightPlan(i).cmd = uint16(0);
    Telem.FlightPlan(i).param1 = single(0);
    Telem.FlightPlan(i).param2 = single(0);
    Telem.FlightPlan(i).param3 = single(0);
    Telem.FlightPlan(i).param4 = single(0);
    Telem.FlightPlan(i).x = int32(0);
    Telem.FlightPlan(i).y = int32(0);
    Telem.FlightPlan(i).z = single(0);
end
% Fence
for i = 1:Telem.NUM_FENCE_POINTS
    Telem.Fence(i).autocontinue = boolean(0);
    Telem.Fence(i).frame = uint8(0);
    Telem.Fence(i).cmd = uint16(0);
    Telem.Fence(i).param1 = single(0);
    Telem.Fence(i).param2 = single(0);
    Telem.Fence(i).param3 = single(0);
    Telem.Fence(i).param4 = single(0);
    Telem.Fence(i).x = int32(0);
    Telem.Fence(i).y = int32(0);
    Telem.Fence(i).z = single(0);
end
% Rally points
for i = 1:Telem.NUM_RALLY_POINTS
    Telem.Rally(i).autocontinue = boolean(0);
    Telem.Rally(i).frame = uint8(0);
    Telem.Rally(i).cmd = uint16(0);
    Telem.Rally(i).param1 = single(0);
    Telem.Rally(i).param2 = single(0);
    Telem.Rally(i).param3 = single(0);
    Telem.Rally(i).param4 = single(0);
    Telem.Rally(i).x = int32(0);
    Telem.Rally(i).y = int32(0);
    Telem.Rally(i).z = single(0);
end

%% Cleanup
clear vehicle fh_vehicle op_point op_report op_spec opt i;
