%Initialize a vehicle for use in the simulation
%
%
% Tuan Luong


%% Platform's name
Aircraft.name = 'ale';

%% Mass properties (Obtained using Solidworks) CG is at body origin
% Mass [kg]
Aircraft.Mass.mass_kg = 1.2;
% c.g. location [m]
Aircraft.Mass.cg_m = [0 0 0];
% Moments of inertia [kg*m^2] obtained from Solidworks model
Aircraft.Mass.ixx_kgm2 = 0.01249536;
Aircraft.Mass.iyy_kgm2 = 0.01309266;
Aircraft.Mass.izz_kgm2 = 0.02338074;
Aircraft.Mass.ixz_kgm2 = 0.014;
Aircraft.Mass.inertia_kgm2 = [Aircraft.Mass.ixx_kgm2    0   -Aircraft.Mass.ixz_kgm2;...
                              0          Aircraft.Mass.iyy_kgm2          0;...
                              -Aircraft.Mass.ixz_kgm2   0       Aircraft.Mass.izz_kgm2];

%% Geometric properties of the body
% Axial area (m^2) in body frame
% Frontal area at different angles
Aircraft.Geo.front_area_m2 = [0.32, 0.36, 0.4, 0.43, 0.45, 0.5, 0.47,...
    0.44, 0.47,0.34,0.47,0.44,0.47,0.5,0.45,0.43,0.4,0.36,0.32,0.36,...
    0.4,0.43,0.45,0.5,0.47,0.44,0.47,0.34,0.47, 0.44, 0.47, 0.5, 0.45, ...
    0.43,0.4,0.36,0.32];

%% Aerodymanics coef
% Axis system for aerodynamic coefficients
% https://www.mathworks.com/help/aeroblks/aerodynamicforcesandmoments.html
% 1 = Wind axis
% 2 = Stability axis
% 3 = Body axis
Aircraft.Aero.axis = 1;
%Drag coefficient
Aircraft.Aero.Cd = 0.8; %Based on CD of slanted cube [Jan Willem Vervoorst]

%% Inceptor configuration
% Configure function of main control channel as well as normalize them
% Other channels are available as raw 172-1811 for FrSky sbus
Aircraft.Inceptor.throttle = 3;
Aircraft.Inceptor.turn = 2;
Aircraft.Inceptor.fire = 6;
Aircraft.Inceptor.mode0 = 5;
Aircraft.Inceptor.throttle_en = 7;
Aircraft.Inceptor.coef_1 = [0.00061013, -0.10494204]; %Coeff for sbus to 0:1
Aircraft.Inceptor.coef_2 = [0.00122026, -1.20988408]; %Coeff for sbus to -1:1
Aircraft.Inceptor.coef_3 = [0.00122026, -0.20988408]; %Coeff for sbus to 0:2

%% Effectors
% Number of PWM channels
Aircraft.Eff.nPwm = 8;
% Number of SBUS channels
Aircraft.Eff.nSbus = 16;
% Total number of channels
Aircraft.Eff.nCh = Aircraft.Eff.nPwm + Aircraft.Eff.nSbus;

%% Propulsion properties
% Number of motors
Aircraft.Motor.nMotor = 2;

% Assign a pwm channel to motor
Aircraft.Motor.map = [ 1 ; 2 ];

% Motor mixing laws [thrust, roll, pitch, yaw]
% The cmd vector [thrust,roll,pitch, yaw] will by multiplied with the motor
% mixing matrix to result in the individual motor outputs which is then
% scaled to the PMW range that the ESC can decode
Aircraft.Motor.mix = [0.95,  0.2, 0, 0; ...
                      0.95, -0.2, 0, 0; ...
                      0,     0,   1, 0; ...
                      0,     0,   0, 1; ...
                      zeros(4, 4)];
                  

%% Battery
% Number of battery cells
Aircraft.Battery.nCell = 3;
% Maximum voltage per cell [V]
Aircraft.Battery.volt_per_cell = 4.2;
% Voltage available
Aircraft.Battery.voltage = Aircraft.Battery.nCell * Aircraft.Battery.volt_per_cell;

%% Sensors (copied from BFS existing model due to same FMS)
% MPU-9250 IMU
% Accel
Aircraft.Sensors.Imu.Accel.scale_factor = eye(3);
Aircraft.Sensors.Imu.Accel.bias_mps2 = [0 0 0]';
Aircraft.Sensors.Imu.Accel.noise_mps2 = 0.0785 * ones(3, 1);
Aircraft.Sensors.Imu.Accel.upper_limit_mps2 = 156.9064 * ones(3, 1);
Aircraft.Sensors.Imu.Accel.lower_limit_mps2 = -1 * Aircraft.Sensors.Imu.Accel.upper_limit_mps2;
% Gyro
Aircraft.Sensors.Imu.Gyro.scale_factor = eye(3);
Aircraft.Sensors.Imu.Gyro.bias_radps = [0 0 0]';
% G-sensitivity in rad/s per m/s/s
Aircraft.Sensors.Imu.Gyro.accel_sens_radps = [0 0 0]';  
Aircraft.Sensors.Imu.Gyro.noise_radps = deg2rad(0.1) * ones(3, 1);
Aircraft.Sensors.Imu.Gyro.upper_limit_radps = deg2rad(2000) * ones(3, 1);
Aircraft.Sensors.Imu.Gyro.lower_limit_radps = -1 * Aircraft.Sensors.Imu.Gyro.upper_limit_radps;
% Magnetometer
Aircraft.Sensors.Imu.Mag.scale_factor = eye(3);
Aircraft.Sensors.Imu.Mag.bias_ut = [0 0 0]';
Aircraft.Sensors.Imu.Mag.noise_ut =  0.6 * ones(3, 1);
Aircraft.Sensors.Imu.Mag.upper_limit_ut =  4800 * ones(3, 1);
Aircraft.Sensors.Imu.Mag.lower_limit_ut = -1 * Aircraft.Sensors.Imu.Mag.upper_limit_ut;
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
% Static pressure
Aircraft.Sensors.PitotStaticInstalled = 0;
Aircraft.Sensors.StaticPres.scale_factor = 1;
Aircraft.Sensors.StaticPres.bias_pa = 0;
Aircraft.Sensors.StaticPres.upper_limit_pa = 120000;
Aircraft.Sensors.StaticPres.lower_limit_pa = 70000;
% 1% of the full-scale range
Aircraft.Sensors.StaticPres.noise_pa = 0.01 * (Aircraft.Sensors.StaticPres.upper_limit_pa - Aircraft.Sensors.StaticPres.lower_limit_pa);
% Differential pressure
Aircraft.Sensors.DiffPres.scale_factor = 1;
Aircraft.Sensors.DiffPres.bias_pa = 0;
Aircraft.Sensors.DiffPres.upper_limit_pa = 1000;
Aircraft.Sensors.DiffPres.lower_limit_pa = 0;
% 2% of the full-scale range
Aircraft.Sensors.DiffPres.noise_pa =  0.02 * (Aircraft.Sensors.DiffPres.upper_limit_pa - Aircraft.Sensors.DiffPres.lower_limit_pa);

%% Controller parameters
% Motor minimum throttle 
% spin motor slowly when armed for safety reasons and anti lock-up
Aircraft.Control.motor_spin_min = 0.0;
Aircraft.Control.off_state = [0, 0, -1, -1, 0, 0, 0, 0];

%% Yaw rate controller parameters
% Max yaw rate [radps]
Aircraft.Control.yaw_rate_max = 0.524; %~30deg/s
% It's good to limit the maximum yaw rate because excessive yaw rate may
% cause some motors to slow down too much that hover cannot be maintained

% Yaw accel PI gains
Aircraft.Control.P_yaw_rate = 0.5;
Aircraft.Control.I_yaw_rate = 0.05;
Aircraft.Control.D_yaw_rate = 0.02;

%% Translational speed controller parameters
% Horizontal spped limit [m/s]
Aircraft.Control.v_hor_max = 5;

% Horizontal speed controller gain
Aircraft.Control.P_v_hor = 0.5;
Aircraft.Control.I_v_hor = 0.01;
Aircraft.Control.D_v_hor = 0.1;

%% Distance controller parameters
Aircraft.Control.P_xy = 3;
Aircraft.Control.I_xy = 0.1;
Aircraft.Control.wp_radius = 1.5;
Aircraft.Control.wp_nav_speed = 3;

%% Heading controller parameters
Aircraft.Control.P_heading = 1;
Aircraft.Control.I_heading = 0.01;
