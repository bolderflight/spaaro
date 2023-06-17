%Initialize a vehicle for use in the simulation
%
%
% Tuan Luong


%% Platform's name
Aircraft.name = 'super';

%% Mass properties (Obtained using Solidworks) CG is at body origin
% Mass [kg]
Aircraft.Mass.mass_kg = 20.4117;
% c.g. location [m]
Aircraft.Mass.cg_m = [0 0 0];
% Moments of inertia [kg*m^2] obtained from Solidworks model
Aircraft.Mass.ixx_kgm2 = 0.07151;
Aircraft.Mass.iyy_kgm2 = 0.08636;
Aircraft.Mass.izz_kgm2 = 0.15364;
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
% Configure function of main control
% Other channels are available as raw 172-1811 for FrSky sbus
Aircraft.Inceptor.throttle = 1;
Aircraft.Inceptor.roll = 2;
Aircraft.Inceptor.pitch = 3;
Aircraft.Inceptor.yaw = 4;
Aircraft.Inceptor.mode0 = 5;
Aircraft.Inceptor.relay = 6;
Aircraft.Inceptor.throttle_e_stop = 7;
Aircraft.Inceptor.engine_cmd = 8;
Aircraft.Inceptor.rtl = 9;

% Inceptor deadband
Aircraft.Inceptor.deadband = 0.1;

%% Effectors
% Number of PWM channels
Aircraft.Eff.nPwm = 8;
% Number of SBUS channels
Aircraft.Eff.nSbus = 16;
% Total number of channels
Aircraft.Eff.nCh = Aircraft.Eff.nPwm + Aircraft.Eff.nSbus;

%% Propulsion properties
% Number of motors
Aircraft.Motor.nMotor = 6;

% Assign a pwm channel to motor
Aircraft.Motor.map = [ 1 ; 2 ; 3 ; 4 ; 5 ; 6 ];

% Motor positions relative to c.g in [m] [x,y,z](obtained from Solidworks)
% Motor numbers and order using Arducopter convention
Aircraft.Motor.pos_m = [0.0   0.78    0;...
                        0.0   -0.78   0;...
                        0.75    -0.43   0;...
                        -0.75   0.43    0;...
                        0.75    0.43    0;...
                        -0.75   -0.43   0]; 

% Alignment with body frame x, y, z axis 
Aircraft.Motor.align = zeros ( Aircraft.Motor.nMotor , 3);
%All motor align with body -z axis for hexacopter case
Aircraft.Motor.align (:,3) = -1; 

% Motor speed constant Kv
Aircraft.Motor.kv = 90; %Kv of T-Motor MN1005

% Motor zero load current [Amp]
Aircraft.Motor.io = 0.9; 

% Motor internal resistance [Ohm]
Aircraft.Motor.r = 0.168;

% Motor rotation direction (right hand rule with z_body)
Aircraft.Motor.dir = [1;-1;1;-1;-1;1];

% Coef of torque of MN1005 motor based on T-Motor's website
Aircraft.Motor.kq = 0.1495;     %N-m/A


% Motor mixing laws [thrust, roll, pitch, yaw]
% The cmd vector [thrust,roll,pitch, yaw] will by multiplied with the motor
% mixing matrix to result in the individual motor outputs which is then
% scaled to the PMW range that the ESC can decode
Aircraft.Motor.motor_yaw_factor = 0.1;
Aircraft.Motor.mix = [0.7,  -0.2,     0, -Aircraft.Motor.motor_yaw_factor;...
                      0.7,   0.2,     0,  Aircraft.Motor.motor_yaw_factor;...
                      0.7,   0.1,  0.2, -Aircraft.Motor.motor_yaw_factor;...
                      0.7,  -0.1, -0.2,  Aircraft.Motor.motor_yaw_factor;...
                      0.7,  -0.1,  0.2, Aircraft.Motor.motor_yaw_factor;...
                      0.7,   0.1, -0.2, -Aircraft.Motor.motor_yaw_factor;
                      0, 0, 0, 0; 
                      0, 0, 0, 0];

    
%% Propeller 
%Diameter [inches]
Aircraft.Prop.dia_in = 32;

% Coefficient of thrust constant obtained from T-motor's website data
Aircraft.Prop.kt = 0.0388;   %N-m/N

%Polynomial coefficient for simple thrust model
Aircraft.Prop.poly_thrust = [109.86 -18.329];
Aircraft.Prop.poly_torque = [4.2625 -0.7112];

%% Battery
% Number of battery cells
Aircraft.Battery.nCell = 12;
% Maximum voltage per cell [V]
Aircraft.Battery.volt_per_cell = 4.2;
% Voltage available
Aircraft.Battery.voltage = Aircraft.Battery.nCell * Aircraft.Battery.volt_per_cell;
% Power module voltage gain. Gain between battery voltage and voltage
% output by power modele
Aircraft.Battery.voltage_gain = 18.95;
% Power module current to voltage gain. Gain between current draw and
% voltage output by power module
Aircraft.Battery.current_to_voltage_gain_vpma = 125.65 * 1000; %mA per volt

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
% allow values to come from telem bus or be hardcoded from this file
Aircraft.Control.hardcode_values = false;

% Motor minimum throttle 
% spin motor slowly when armed for safety reasons and anti lock-up
Aircraft.Control.motor_spin_min = 0.15; 

% Thorttle stick minimum
% Mimimum valid value of the throttle stick so that arming and gain reset
% occurs
Aircraft.Control.throttle_min = 0.05;

% Motor ramp time [s]
% Time so slowly ramp motor from 0 to motor_spin_min. Prevent initial
% voltage spike
Aircraft.Control.motor_ramp_time_s = 3;

%% Yaw rate controller parameters
% Max yaw rate [radps]
Aircraft.Control.yaw_rate_max = 1.74533; %~100deg/s
% It's good to limit the maximum yaw rate because excessive yaw rate may
% cause some motors to slow down too much that hover cannot be maintained

% Yaw accel PI gains
Aircraft.Control.P_yaw_rate = 0.5;
Aircraft.Control.I_yaw_rate = 0.05;
Aircraft.Control.D_yaw_rate = 0.02;

%% Pitch controller parameters
% Max pitch angle [rad]
Aircraft.Control.pitch_angle_lim = 0.35;  %~20deg

% Pitch cmd controller gains
Aircraft.Control.P_pitch_angle = 0.04;
Aircraft.Control.I_pitch_angle = 0.04;
Aircraft.Control.D_pitch_angle = 0.02;

% Max pitch rate [radps]
Aircraft.Control.pitch_rate_max = 1; %~60deg/s

%% Roll controller parameters
% Max roll angle [rad]
Aircraft.Control.roll_angle_lim = 0.35;  %~20deg

% Roll cmd controller gains
Aircraft.Control.P_roll_angle = 0.04;
Aircraft.Control.I_roll_angle = 0.04;
Aircraft.Control.D_roll_angle = 0.02;

% Max roll rate [radps]
Aircraft.Control.roll_rate_max = 1; %~60deg/s

%% Vertical speed controller parameters
Aircraft.Control.est_hover_thr = 0.6724;
% Vertical speed limit [m/s]
Aircraft.Control.v_z_up_max = 2;
Aircraft.Control.v_z_down_max = 1; %minimum of -1 m/s
% Vertical speed controller gain
Aircraft.Control.P_v_z = 0.09;
Aircraft.Control.I_v_z = 0.05;
Aircraft.Control.D_v_z = 0.005;

%% Translational speed controller parameters
% Horizontal spped limit [m/s]
Aircraft.Control.v_hor_max = 5;

% Horizontal speed controller gain
Aircraft.Control.P_v_hor = 0.5;
Aircraft.Control.I_v_hor = 0.01;
Aircraft.Control.D_v_hor = 0.1;


%% Altitude controller parameters
Aircraft.Control.P_alt = 1;
Aircraft.Control.I_alt = 0.2;

%% Distance controller parameters
Aircraft.Control.P_xy = 3;
Aircraft.Control.I_xy = 0.1;
Aircraft.Control.wp_radius = 1.5;
Aircraft.Control.wp_nav_speed = 3;

%% Heading controller parameters
Aircraft.Control.P_heading = 1;
Aircraft.Control.I_heading = 0.01;
Aircraft.Control.D_heading = 0.01;

%% Return to land parameters
Aircraft.Control.rtl_altitude = 35;
Aircraft.Control.land_speed_fast = 1;
Aircraft.Control.land_speed_slow = 0.3;
Aircraft.Control.land_slow_alt = 10; % Altitude [m] where precision landing is engaged