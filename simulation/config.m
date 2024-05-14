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
% vehicle = 'malt';
vehicle = 'session_v0';
% vehicle = 'super';
% vehicle = 'lambu';

% FMU-R version
if strcmpi(vehicle, 'malt') || strcmpi(vehicle, 'lambu')
    fmu_version = "mini";
elseif strcmpi(vehicle, 'ale')
    fmu_version = "v1";
elseif any(strcmpi(vehicle, {'session_v0', 'super'}))
    fmu_version = "v2";
else
    ME = MException("UASpaaro:noSuchVehicle", sprintf("No vehicle %s", vehicle));
    throw(ME);
end

vms_only = false;


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

Env.const_g = 9.81;

%% Initial Environmental Conditions

% Initial lat, lon in rad
InitCond.lat_rad = 0.4831;
InitCond.lon_rad = 1.4735;

InitCond.alt_m = 67.117600;

% [Xe, Ye, Ze]
InitCond.ned_pos_m = [0 0 -100];

% [u, v, w]
InitCond.body_vel_mps = [0.5, 0.1, 0.1];

% [roll, pitch, yaw]
InitCond.euler_rad = [0 0 0];

% [p, q, r]
InitCond.body_rot_rate_radps = [0 0 0];

%% Additional Effects
% Bool of whether to include additional effects. With some parameters

% Effects of wind
AddEffects.wind.steady_bool = 0; 
AddEffects.wind.gust_bool = 1;
AddEffects.wind.direction_rad = 0.6;
AddEffects.wind.speed_mps = 4; 

% Noise and bias on the angular rate measurements
AddEffects.pqr.noise_bool = 0;
AddEffects.pqr.bias_bool = 0;

% Effects to aerodynamic moment coefficients
AddEffects.aero_moments.loss_control_eff_bias_bool = 0;
AddEffects.aero_moments.sys_dynamics_bias_bool = 0; 
AddEffects.aero_moments.percent_diff = 10;

% Effects to body moment of inertia
AddEffects.inertia.bias_bool = 0;
AddEffects.inertia.percent_diff = 10;

% Effects to hover propulsive performance
AddEffects.hover_prop.bias_bool = 0;
AddEffects.hover_prop.percent_diff = 5;

% Noise on the Airspeed sensor
AddEffects.airspeed.noise_bool = 0;

% Actuator faults and loss of control effectiveness
AddEffects.actuator_fault_bool = 0; 
AddEffects.elevator_stuck_bool = 0;
AddEffects.surf_fault_scale = 0.75;
AddEffects.motor_fault_scale_1 = 0.95;
AddEffects.motor_fault_scale_2 = 0.85;

% Actuator dynamics uncertainty
AddEffects.motor_bias_bool = 0;
AddEffects.surf_bias_bool = 0;






