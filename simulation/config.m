% Configures the simulation
%
% Brian R Taylor
% brian.taylor@bolderflight.com
% 
% Copyright (c) 2021 Bolder Flight Systems
%

%% Definitions
% Vehicle
% vehicle = 'sig_kadet';
 vehicle = 'malt';
%vehicle = 'session_v0';
% vehicle = 'super';

% FMU-R version
if strcmpi(vehicle, 'malt')
    fmu_version = "mini";
elseif strcmpi(vehicle, 'ale')
    fmu_version = "v1";
elseif any(strcmpi(vehicle, {'session_v0', 'super'}))
    fmu_version = "v2";
else
    ME = MException("UASpaaro:noSuchVehicle", sprintf("No vehicle %s", vehicle));
    throw(ME);
end

vms_only = true;

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
Env.terrain_alt_msl_m = 0;
Env.terrain_alt_wgs84 = 0;

%% Initial Environmental Conditions

% Initial lat, lon in rad
InitCond.lat_rad = 0.579621767644;
InitCond.lon_rad = -1.527761279167;

InitCond.alt_m = 67.117600;

% [Xe, Ye, Ze]
InitCond.ned_pos_m = [0 0 -200];

% [u, v, w]
InitCond.body_vel_mps = [50 0 0];

% [roll, pitch, yaw]
InitCond.euler_rad = [0 0 0];

% [p, q, r]
InitCond.body_rot_rate_radps = [0 0 0];


