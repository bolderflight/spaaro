% Configures the simulation for the Sig Kadet LT-40
%
% Brian R Taylor
% brian.taylor@bolderflight.com
% 
% Copyright (c) 2021 Bolder Flight Systems
%

% Aircraft name
Aircraft.name = 'SigKadet';

%% Control law
% Control surface position limits
Control.elev_lim_deg = 20;
Control.ail_lim_deg = 20;
Control.rud_lim_deg = 20;
Control.pla_lim_min = 0;
Control.pla_lim_max = 1;
% Stick to reference angle mapping
Control.Attitude.Pitch.ref_deg = 30;
Control.Attitude.Roll.ref_deg = 45;
% Reference airspeed, m/s
Control.Airspeed.ref_mps = 17;
% PID gains
Control.Attitude.Pitch.p = 0.8;
Control.Attitude.Pitch.i = 0.2;
Control.Attitude.Pitch.d = 0.15;

Control.Attitude.Roll.p = 0.5;
Control.Attitude.Roll.i = 0.2;
Control.Attitude.Roll.d = 0.15;

Control.Airspeed.p = 0.09;
Control.Airspeed.i = 0.02;

Control.Altitude.p = 0.02;
Control.Altitude.i = 0.002;

Control.pitch_lim_deg = 10;
Control.roll_lim_deg = 30;
Control.lat_rate_lim_mps = 10;

%% Mass properties
% Mass, kg
Aircraft.Mass.mass_kg = 1.959;
% c.g. location [x y z], m
Aircraft.Mass.cg_m = [0.222 0 0.046];
% Moments of inertia, kg*m^2
Aircraft.Mass.ixx_kgm2 = 0.07151;
Aircraft.Mass.iyy_kgm2 = 0.08636;
Aircraft.Mass.izz_kgm2 = 0.15364;
Aircraft.Mass.ixz_kgm2 = 0.014;
Aircraft.Mass.inertia_kgm2 = ...
    [Aircraft.Mass.ixx_kgm2    0   -Aircraft.Mass.ixz_kgm2;...
              0          Aircraft.Mass.iyy_kgm2          0;...
     -Aircraft.Mass.ixz_kgm2   0       Aircraft.Mass.izz_kgm2];
                        
%% Geometric parameters
% Reference chord, m
Aircraft.Geom.c_m = 0.25;
% Reference span, m
Aircraft.Geom.b_m = 1.27;
% Reference area, m^2
Aircraft.Geom.s_m2 = 0.3097;
% Center of pressure [x y z], m
Aircraft.Geom.cp_m = [0.2175 0 0.046];

%% Effectors
% Number of PWM channels
Aircraft.Eff.nPwm = 8;
% Number of SBUS channels
Aircraft.Eff.nSbus = 16;
% Total number of channels
Aircraft.Eff.nCh = Aircraft.Eff.nPwm + Aircraft.Eff.nSbus;

%% Control surfaces
% Number of control surfaces
% [left ail, right ail, elev, ruder]
% Positive TED, TEL
Aircraft.Surf.nSurf = 4;
% PWM / SBUS channel number to control surface mapping
% Array ordered as 8 PWM channels then 16 SBUS channels for a total of 24
% channels, 1-based indexing
Aircraft.Surf.map = [11 12 9 10];
% Rate limits
Aircraft.Surf.Limit.rate_dps = 150 * ones(Aircraft.Surf.nSurf, 1);
% Position limits
Aircraft.Surf.Limit.pos_deg = 25 * ones(Aircraft.Surf.nSurf, 1);
Aircraft.Surf.Limit.neg_deg = -25 * ones(Aircraft.Surf.nSurf, 1);

%% Aerodynamic paramters
% Axis system for aerodynamic coefficients
% https://www.mathworks.com/help/aeroblks/aerodynamicforcesandmoments.html
% 1 = Wind axis
% 2 = Stability axis
% 3 = Body axis
Aircraft.Aero.axis = 1;
% Angle of attack breakpoints for control effectiveness
Aircraft.Aero.CntrlEff.alpha = [-20 20];
% X force
Aircraft.Aero.CX.zero = 0.0434;
Aircraft.Aero.CX.alpha = 1.5;
Aircraft.Aero.CX.q = 3.1010;
% Control effectivess in the CX axis, build up with alpha breakpoints
Aircraft.Aero.CX.surf(1, :) = [0 0 0 0];
Aircraft.Aero.CX.surf(2, :) = [0 0 0 0];
% Y force
Aircraft.Aero.CY.zero = 0;
Aircraft.Aero.CY.beta = -0.4889;
Aircraft.Aero.CY.p = -0.0375;
Aircraft.Aero.CY.r = 0.1500;
% Control effectivess in the CY axis, build up with alpha breakpoints
Aircraft.Aero.CY.surf(1, :) = [0 0 0 -0.0303];
Aircraft.Aero.CY.surf(2, :) = [0 0 0 -0.0303];
% Z force
Aircraft.Aero.CZ.zero = 0.1086;
Aircraft.Aero.CZ.alpha = 4.58;
Aircraft.Aero.CZ.q = 6.1639;
% Control effectivess in the CZ axis, build up with alpha breakpoints
Aircraft.Aero.CZ.surf(1, :) = [0 0 -0.0983 0];
Aircraft.Aero.CZ.surf(2, :) = [0 0 -0.0983 0];
% Roll moment
Aircraft.Aero.Cl.zero = 0;
Aircraft.Aero.Cl.beta = -0.0545;
Aircraft.Aero.Cl.p = -0.4496;
Aircraft.Aero.Cl.r = 0.1086;
% Control effectivess in the Cl axis, build up with alpha breakpoints
Aircraft.Aero.Cl.surf(1, :) = [0.1646/2 0.1646/2 0 -0.0115];
Aircraft.Aero.Cl.surf(2, :) = [0.1646/2 0.1646/2 0 -0.0115];
% Pitch moment
Aircraft.Aero.Cm.zero = -0.0278;
Aircraft.Aero.Cm.alpha = -0.7230;
Aircraft.Aero.Cm.q = -13.5664;
% Control effectivess in the Cm axis, build up with alpha breakpoints
Aircraft.Aero.Cm.surf(1, :) = [0 0 0.8488 0];
Aircraft.Aero.Cm.surf(2, :) = [0 0 0.8488 0];
% Yaw moment
Aircraft.Aero.Cn.zero = 0;
Aircraft.Aero.Cn.beta = 0.0723;
Aircraft.Aero.Cn.p = 0.1180;
Aircraft.Aero.Cn.r = -0.1833;
% Control effectivess in the Cn axis, build up with alpha breakpoints
Aircraft.Aero.Cn.surf(1, :) = [0 0 0 0.1811];
Aircraft.Aero.Cn.surf(2, :) = [0 0 0 0.1811];

%% Battery
% Number of battery cells
Aircraft.Battery.nCell = 3;
% Maximum voltage per cell
Aircraft.Battery.volt_per_cell = 4.2;
% Voltage available
Aircraft.Battery.voltage = Aircraft.Battery.nCell * ...
    Aircraft.Battery.volt_per_cell;

%% Motor
% Number of motors
Aircraft.Motor.nMotor = 1;
% PWM / SBUS channel number to motor mapping
% Array ordered as 8 PWM channels then 16 SBUS channels for a total of 24
% channels, 1-based indexing
Aircraft.Motor.map = [1];
% Position relative to c.g., m
Aircraft.Motor.pos_m(1, :) = [-0.075 0 0];
% Alignment with x, y, z axis
Aircraft.Motor.align(1, :) = [1 0 0];
% Speed constant, Kv [RPM/V]
Aircraft.Motor.kv = 870;
% Resistance [ohm]
Aircraft.Motor.r = 0.03;
% Zero torque current [Amp]
Aircraft.Motor.io = 2.4;

%% Propeller
% Diameter [inches]
Aircraft.Prop.dia_in = 12;
% Coefficient of thrust polynomial coefficients
Aircraft.prop.ct = [-2.4822 4.1010 -2.6695 0.7331 -0.1958 0.0978];
% Coefficient of power polynomial coefficients
Aircraft.prop.cp = [-1.8863 2.5393 -1.3781 0.3089 -0.0358 0.0329];
% Electric motor and propeller combine moment of inertia [kg*m^2]
Aircraft.Prop.Jmp_kgm2 = 0.00012991;


%% Sensors
% MPU-9250 IMU
% Accel
Aircraft.Sensors.Imu.Accel.scale_factor = eye(3);
Aircraft.Sensors.Imu.Accel.bias_mps2 = [0 0 0]';
Aircraft.Sensors.Imu.Accel.noise_mps2 = 0.0785 * ones(3, 1);
Aircraft.Sensors.Imu.Accel.upper_limit_mps2 = 156.9064 * ones(3, 1);
Aircraft.Sensors.Imu.Accel.lower_limit_mps2 = -1 * ...
    Aircraft.Sensors.Imu.Accel.upper_limit_mps2;
% Gyro
Aircraft.Sensors.Imu.Gyro.scale_factor = eye(3);
Aircraft.Sensors.Imu.Gyro.bias_radps = [0 0 0]';
% G-sensitivity in rad/s per m/s/s
Aircraft.Sensors.Imu.Gyro.accel_sens_radps = [0 0 0]';  
Aircraft.Sensors.Imu.Gyro.noise_radps = deg2rad(0.1) * ones(3, 1);
Aircraft.Sensors.Imu.Gyro.upper_limit_radps = deg2rad(2000) * ones(3, 1);
Aircraft.Sensors.Imu.Gyro.lower_limit_radps = -1 * ...
    Aircraft.Sensors.Imu.Gyro.upper_limit_radps;
% Magnetometer
Aircraft.Sensors.Imu.Mag.scale_factor = eye(3);
Aircraft.Sensors.Imu.Mag.bias_ut = [0 0 0]';
Aircraft.Sensors.Imu.Mag.noise_ut =  0.6 * ones(3, 1);
Aircraft.Sensors.Imu.Mag.upper_limit_ut =  4800 * ones(3, 1);
Aircraft.Sensors.Imu.Mag.lower_limit_ut = -1 * ...
    Aircraft.Sensors.Imu.Mag.upper_limit_ut;
% GNSS model
Aircraft.Sensors.Gnss.sample_rate_hz = 5;
Aircraft.Sensors.Gnss.fix = 3; % 3D fix
Aircraft.Sensors.Gnss.num_satellites = 16;
Aircraft.Sensors.Gnss.horz_accuracy_m = 1.5;
Aircraft.Sensors.Gnss.vert_accuracy_m = 5.5;
Aircraft.Sensors.Gnss.vel_accuracy_mps = 0.05;
Aircraft.Sensors.Gnss.track_accuracy_rad = deg2rad(2);
Aircraft.Sensors.Gnss.hdop = 0.7;
Aircraft.Sensors.Gnss.vdop = 0.7;
% Air data model
Aircraft.Sensors.PitotStaticInstalled = 1;
% Static pressure
Aircraft.Sensors.StaticPres.scale_factor = 1;
Aircraft.Sensors.StaticPres.bias_pa = 0;
Aircraft.Sensors.StaticPres.upper_limit_pa = 120000;
Aircraft.Sensors.StaticPres.lower_limit_pa = 70000;
% 1% of the full-scale range
Aircraft.Sensors.StaticPres.noise_pa = 0.01 * ...
    (Aircraft.Sensors.StaticPres.upper_limit_pa - ...
    Aircraft.Sensors.StaticPres.lower_limit_pa);
% Pitot differential pressure
Aircraft.Sensors.DiffPres.scale_factor = 1;
Aircraft.Sensors.DiffPres.bias_pa = 0;
Aircraft.Sensors.DiffPres.upper_limit_pa = 1000;
Aircraft.Sensors.DiffPres.lower_limit_pa = 0;
% 2% of the full-scale range
Aircraft.Sensors.DiffPres.noise_pa =  0.02 * ...
    (Aircraft.Sensors.DiffPres.upper_limit_pa - ...
    Aircraft.Sensors.DiffPres.lower_limit_pa);
