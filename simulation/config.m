% Configures the simulation
%
% Brian R Taylor
% brian.taylor@bolderflight.com
% 
% Copyright (c) 2021 Bolder Flight Systems
%

%% Target trim conditions
% Latitude and longitude [deg]
Target.lat_deg = 35.691544;
Target.lon_deg = -105.944183;
% Altitude above mean sea level [m]
Target.alt_msl_m = 100;
% Heading [deg] and airspeed [m/s]
Target.heading_deg = 90;  % +/-180 degrees relative to true north
Target.airspeed_mps = 17;
% Ground height above mean sea level [m]
Env.terrain_alt_wgs84 = 0;
Env.wmm_nt = [22938.3 -33.7 -132.6];

%% Initial Condition for simulation
InitCond.ned_pos_m = [0 0 -5];
InitCond.body_vel_mps = [1 0 0];
InitCond.euler_rad = [0 0 0];
InitCond.body_rot_rate_radps = [0 0 0];
InitCond.lat_rad = 0.579621767644;
InitCond.lon_rad = -1.527761279167;

%% Flight software frame rate
frameRate_hz = 50;
framePeriod_s = 1/frameRate_hz;

%% Definitions
% Vehicle
% vehicle = 'super';
% vehicle = 'queso';
vehicle = 'ale';

% FMU-R version
% fmu_version = "v1";
% fmu_version = "v2-beta";
% fmu_version = "v2";
fmu_version = "v1";