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
addpath(genpath('libraries'));
addpath(genpath('matlab'));
addpath(genpath('models'));
addpath(genpath('sensor_processing'));
addpath(genpath('sfun'));
addpath(genpath('vms'));

%% Load bus definitions
load('bus_defs.mat');

%% Call the setup scripts
fh_vehicle = str2func(vehicle);
fh_vehicle();

%% Trim
trim();

%% Cleanup
clear vehicle fh_vehicle;
