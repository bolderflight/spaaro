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
NUM_FLIGHT_PLAN_POINTS = 100;
NUM_FENCE_POINTS = 50;
NUM_RALLY_POINTS = 10;
% Flight plan
for i = 1:NUM_FLIGHT_PLAN_POINTS
    FlightPlan(i).autocontinue = boolean(0);
    FlightPlan(i).frame = uint8(0);
    FlightPlan(i).cmd = uint16(0);
    FlightPlan(i).param1 = single(0);
    FlightPlan(i).param2 = single(0);
    FlightPlan(i).param3 = single(0);
    FlightPlan(i).param4 = single(0);
    FlightPlan(i).x = int32(0);
    FlightPlan(i).y = int32(0);
    FlightPlan(i).z = single(0);
end
% Fence
for i = 1:NUM_FENCE_POINTS
    Fence(i).autocontinue = boolean(0);
    Fence(i).frame = uint8(0);
    Fence(i).cmd = uint16(0);
    Fence(i).param1 = single(0);
    Fence(i).param2 = single(0);
    Fence(i).param3 = single(0);
    Fence(i).param4 = single(0);
    Fence(i).x = int32(0);
    Fence(i).y = int32(0);
    Fence(i).z = single(0);
end
% Rally points
for i = 1:NUM_RALLY_POINTS
    Rally(i).autocontinue = boolean(0);
    Rally(i).frame = uint8(0);
    Rally(i).cmd = uint16(0);
    Rally(i).param1 = single(0);
    Rally(i).param2 = single(0);
    Rally(i).param3 = single(0);
    Rally(i).param4 = single(0);
    Rally(i).x = int32(0);
    Rally(i).y = int32(0);
    Rally(i).z = single(0);
end

%% Cleanup
clear vehicle fh_vehicle;
