% Trims simulation
%
% Brian R Taylor
% brian.taylor@bolderflight.com
% 
% Copyright (c) 2021 Bolder Flight Systems
%

%% Initial conditions
% Initial AGL altitude
Target.alt_agl_m = Target.alt_msl_m - Env.terrain_alt_msl_m;
% Terrain altitude above WGS84
Env.terrain_alt_wgs84 = Env.terrain_alt_msl_m + geoidheight(Target.lat_deg, constrain360(Target.lon_deg), 'EGM2008');
% World Magnetic Model
Env.wmm_nt = wrldmagm(Target.alt_msl_m + geoidheight(Target.lat_deg, constrain360(Target.lon_deg), 'EGM2008'), Target.lat_deg, Target.lon_deg, decyear(now));
% Initial position, flat earth [Xe, Ye, Ze]
InitCond.ned_pos_m = [0 0 -Target.alt_agl_m];
% Initial velocity in Body axes [U, v, w]
InitCond.body_vel_mps = [Target.airspeed_mps 0 0];
% Initial Euler orientation [roll, pitch, yaw]
% Set yaw to 155 deg and then set to the desired heading after trimming is
% complete; otherwise, some directions can be problematic
InitCond.euler_rad = [0 0 deg2rad(155)];
% Initial body rotation rates [p, q, r]
InitCond.body_rot_rate_radps = [0 0 0];
% Initial Latitude and longitude
InitCond.lat_rad = deg2rad(Target.lat_deg);
InitCond.lon_rad = deg2rad(Target.lon_deg);
% Initial engine speed
InitCond.engine_speed_radps = 827 * ones(Aircraft.Motor.nMotor, 1);  % Initial guess to avoid divide by zero

%% Create operating point specifications for sim
op_spec = operspec('trim_sim');

%% State specifications
%op_spec.States(1).StateName = 'omega_engine';
op_spec.States(1).Known       = 0;
op_spec.States(1).x           = InitCond.engine_speed_radps;
op_spec.States(1).steadystate = 1;

% phi, theta, psi -- Euler angles
% op_spec.States(2).StateName = '[phi theta psi]';
op_spec.States(2).Known       = [0; 0; 1];
op_spec.States(2).x           = InitCond.euler_rad';
op_spec.States(2).steadystate = [1; 1; 1];

% p, q, r -- Angular rates
% op_spec.States(3).StateName = '[p q r]';
op_spec.States(3).Known       = [0; 0; 0];
op_spec.States(3).x           = InitCond.body_rot_rate_radps';
op_spec.States(3).steadystate = [1; 1; 1];

% u, v, w -- Body velocities
% op_spec.States(4).StateName = '[u v w]';
op_spec.States(4).Known       = [0; 0; 0];
op_spec.States(4).x           = InitCond.body_vel_mps';
op_spec.States(4).steadystate = [1; 1; 1];
op_spec.States(4).min         = [0 -inf -inf];

% Xe, Ye, Ze -- Inertial position
%op_spec.States(5).StateName  = '[Xe Ye Ze]';
op_spec.States(5).Known       = [0; 0; 0];
op_spec.States(5).x           = InitCond.ned_pos_m';
op_spec.States(5).steadystate = [0; 0; 0];
op_spec.States(5).max         = [inf inf 0];

%% Output specifications
% Airspeed, m/s
op_spec.Outputs(1).Known = 1;
op_spec.Outputs(1).y = Target.airspeed_mps;
% Sideslip angle, rad
op_spec.Outputs(2).Known = 1;
op_spec.Outputs(2).y = 0;
% Flight path angle, rad
op_spec.Outputs(3).Known = 1;
op_spec.Outputs(3).y = 0;
% AGL altitude, m
op_spec.Outputs(4).Known = 1;
op_spec.Outputs(4).y = Target.alt_agl_m;

%% Input specifications
% Control surface positions
op_spec.Inputs(1).Known = zeros(Aircraft.Surf.nSurf, 1);
op_spec.Inputs(1).u = zeros(Aircraft.Surf.nSurf, 1);
op_spec.Inputs(1).max = deg2rad(Aircraft.Surf.Limit.pos_deg);
op_spec.Inputs(1).min = deg2rad(Aircraft.Surf.Limit.neg_deg);
% Motors
op_spec.Inputs(2).Known = zeros(Aircraft.Motor.nMotor, 1);
op_spec.Inputs(2).u = 0.559 * ones(Aircraft.Motor.nMotor, 1);  % Initial guess to avoid divide by zero
op_spec.Inputs(2).min = zeros(Aircraft.Motor.nMotor, 1);

%% Find trim
opt = findopOptions('OptimizationOptions', optimset('MaxFunEvals', 1e+04, 'Algorithm', 'interior-point'), 'DisplayReport', 'on');  
[op_point, op_report] = findop('trim_sim', op_spec, opt);

%% Set sim to trim condition
% States
InitCond.engine_speed_radps = op_point.States(1).x;
InitCond.euler_rad = op_point.States(2).x;
InitCond.body_rot_rate_radps = op_point.States(3).x;
InitCond.body_vel_mps = op_point.States(4).x;
InitCond.ned_pos_m = op_point.States(5).x;

% Set the heading angle correctly
InitCond.euler_rad(3) = deg2rad(Target.heading_deg);

% Pull control surface values for trim solution
InitCond.surf_rad = op_point.Inputs(1).u;

% Pull motor values for trim solution
InitCond.motor_cmd = op_point.Inputs(2).u;
