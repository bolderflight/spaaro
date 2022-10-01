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
clear;
clc;

%% Configure
run('./config');

%% Add paths
addpath(genpath('aircraft'));
addpath(genpath('data'));
addpath(genpath('matlab'));
addpath(genpath('models'));
addpath(genpath('vms'));

%% Specify root folders for autocode and cache
% clear all cached code
cacheFolder = '../flight_code/build/slprj';
codeGenFolder = '../flight_code/autocode';
if exist(cacheFolder, 'dir')
    rmdir(cacheFolder, 's');
end
if exist(codeGenFolder, 'dir')
    rmdir(codeGenFolder, 's');
end
Simulink.fileGenControl('set', ...
    'CacheFolder', cacheFolder, ...
    'CodeGenFolder', codeGenFolder, ...
    'CodeGenFolderStructure', ...
    Simulink.filegen.CodeGenFolderStructure.ModelSpecific, ...
    'createDir', true);

%% Setup aircraft
run(strcat('./aircraft/', vehicle));

%% Setup FMU
%% Setup the flight management unit
if strcmpi(fmu_version, "V2")
    Fmu.version = 3;
    Fmu.NUM_AIN = 8;
    frameRate_hz = 100;
    Telem.NUM_FLIGHT_PLAN_POINTS = 500;
    Telem.NUM_FENCE_POINTS = 100;
    Telem.NUM_RALLY_POINTS = 10;
    load('./data/fmu_v2_bus_defs.mat');
elseif strcmpi(fmu_version, "V2-BETA")
    Fmu.version = 2;
    Fmu.NUM_AIN = 8;
    frameRate_hz = 100;
    Telem.NUM_FLIGHT_PLAN_POINTS = 500;
    Telem.NUM_FENCE_POINTS = 100;
    Telem.NUM_RALLY_POINTS = 10;
    load('./data/fmu_v2_beta_bus_defs.mat');
else
    Fmu.version = 1;
    Fmu.NUM_AIN = 2;
    frameRate_hz = 50;
    Telem.NUM_FLIGHT_PLAN_POINTS = 100;
    Telem.NUM_FENCE_POINTS = 50;
    Telem.NUM_RALLY_POINTS = 10;
    load('./data/fmu_v1_bus_defs.mat');
end
framePeriod_s = 1/frameRate_hz;

%% Trim
%trim();

%% Create flight plan, fence, and rally point structs
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

Telem.FlightPlan = load_waypoint(Telem.FlightPlan,'test_mission_upload.waypoints');

%% Select sim
multirotor_sim
%ground_sim

%% Cleanup
%clear vehicle fh_vehicle op_point op_report op_spec opt i;