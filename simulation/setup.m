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
    'CacheFolder', '../build/slprj', ...
    'CodeGenFolder', '../autocode', ...
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

%% Cleanup
clear vehicle fh_vehicle;
