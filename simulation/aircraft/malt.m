%Initialize a vehicle for use in the simulation
%
%
% Tuan Luong


%% Platform's name
Aircraft.name = 'malt';

%% Mass properties (Obtained using Solidworks) CG is at body origin
% Mass [kg]
Aircraft.Mass.mass_kg = 0.3;
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
Aircraft.Inceptor.throttle_e_stop = 7;

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
Aircraft.Motor.nMotor = 4;

% Assign a pwm channel to motor
Aircraft.Motor.map = [ 1 ; 2 ; 3 ; 4 ];

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
%All motor align with body -z axis for quadcopter case
Aircraft.Motor.align (:,3) = -1; 

% Motor rotation direction (right hand rule with z_body)
Aircraft.Motor.dir = [1; 1; -1; -1];

% Motor mixing laws [thrust, roll, pitch, yaw]
% The cmd vector [thrust,roll,pitch, yaw] will by multiplied with the motor
% mixing matrix to result in the individual motor outputs which is then
% scaled to the PMW range that the ESC can decode
Aircraft.Motor.mix = [0.75, -0.1,  0.1, -0.2;...
                      0.75,  0.1, -0.1, -0.2;...
                      0.75,  0.1,  0.1,  0.2;...
                      0.75, -0.1, -0.1,  0.2;...
                      0, 0, 0, 0;...
                      0, 0, 0, 0;...
                      0, 0, 0, 0;... 
                      0, 0, 0, 0];

    
%% Propeller 
%Diameter [inches]
Aircraft.Prop.dia_in = 3;

% Coefficient of thrust constant obtained from T-motor's website data
Aircraft.Prop.kt = 0.0388;   %N-m/N

%Polynomial coefficient for simple thrust model
Aircraft.Prop.poly_thrust = [109.86 -18.329];
Aircraft.Prop.poly_torque = [4.2625 -0.7112];

%% Battery
% Number of battery cells
Aircraft.Battery.nCell = 2;
% Maximum voltage per cell [V]
Aircraft.Battery.volt_per_cell = 4.2;
% Voltage available
Aircraft.Battery.voltage = Aircraft.Battery.nCell * Aircraft.Battery.volt_per_cell;
% Power module voltage gain. Gain between battery voltage and voltage
% output by power modele
Aircraft.Battery.voltage_gain = 1;
% Power module current to voltage gain. Gain between current draw and
% voltage output by power module
Aircraft.Battery.current_to_voltage_gain_vpma = 200; %mA per volt

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
Aircraft.Control.motor_spin_min = 0.15; 

% Motor maximum throttle
% Prevent motor from spining at max to reduce current draw on the top end
Aircraft.Control.motor_spin_max = 0.95;


%% Yaw rate controller parameters
% Max yaw rate [radps]
Aircraft.Control.yaw_rate_max = 3.14159; %~180deg/s
% It's good to limit the maximum yaw rate because excessive yaw rate may
% cause some motors to slow down too much that hover cannot be maintained

% Yaw rate controller PID gains
Aircraft.Control.P_yaw_rate = 0.3;
Aircraft.Control.I_yaw_rate = 0.1;
Aircraft.Control.D_yaw_rate = 0.01;

% Yaw rate D Controller LPFT cutoff frequency [Hz]
Aircraft.Control.D_yaw_FLTR_CTOFF = 10;

%% Pitch rate controller parameters
% Max pitch rate [radps]
Aircraft.Control.pitch_rate_max = 3.83972; %~220deg/s

% Pitch rate controller PID gains
Aircraft.Control.P_pitch_rate = 0.25;
Aircraft.Control.I_pitch_rate = 0.2;
Aircraft.Control.D_pitch_rate = 0.01;

% Pitch rate D Controller LPFT cutoff frequency [Hz]
Aircraft.Control.D_pitch_FLTR_CTOFF = 10;

%% Roll rate controller parameters
% Max roll rate [radps]
Aircraft.Control.roll_rate_max = 3.83972; %~220deg/s

% Roll rate controller PID gains
Aircraft.Control.P_roll_rate = 0.25;
Aircraft.Control.I_roll_rate = 0.3;
Aircraft.Control.D_roll_rate = 0.01;

% Roll rate D Controller LPFT cutoff frequency [Hz]
Aircraft.Control.D_roll_FLTR_CTOFF = 10;

%% Pitch angle controller parameters
% Max pitch angle [rad]
Aircraft.Control.pitch_angle_lim = 0.261799;  %~15deg

% Pitch cmd controller gains
Aircraft.Control.P_pitch_angle = 7;

%% Roll controller parameters
% Max roll angle [rad]
Aircraft.Control.roll_angle_lim = 0.261799;  %~15deg

% Roll cmd controller gains
Aircraft.Control.P_roll_angle = 7;

%% Outer to inner loop conversion
% Max G by the drone 
Aircraft.Control.max_g_thr = 1.8;

% Min G by the drone
Aircraft.Control.min_g_thr = 0.1;

% Max tilt angle
Aircraft.Control.max_tilt_rad = deg2rad(25);

%% Vertical speed controller parameters
Aircraft.Control.est_hover_thr = 0.75;
% Vertical speed limit [m/s]
Aircraft.Control.v_z_max = 0.4;
% Vertical speed controller gain
Aircraft.Control.P_v_z = 0.6;
Aircraft.Control.I_v_z = 0.2;
Aircraft.Control.D_v_z = 0.07;

% Ver speed D Controller LPFT cutoff frequency [Hz]
Aircraft.Control.D_ver_vel_FLTR_CTOFF = 1;

% Throttle cc LPFT cutoff frequency [Hz]
Aircraft.Control.throttle_cc_FLTR_CTOFF = 1;

%% Translational speed controller parameters
% Horizontal spped limit [m/s]
Aircraft.Control.v_hor_max = 0.5;

% Horizontal speed controller gain
Aircraft.Control.P_v_hor = 0.5;
Aircraft.Control.I_v_hor = 0.01;
Aircraft.Control.D_v_hor = 0.01;

% Hor speed D Controller LPFT cutoff frequency [Hz]
Aircraft.Control.D_hor_vel_FLTR_CTOFF = 10;

%% Altitude controller parameters
Aircraft.Control.P_alt = 0.7;

%% Distance controller parameters
Aircraft.Control.P_xy = 0.8;
Aircraft.Control.wp_radius = 0.1;

%% WP controller parameters
Aircraft.Control.P_wp_dist = 0.4;
Aircraft.Control.P_dev_dist = 0.5;
Aircraft.Control.nav_speed = 0.3;

%% Heading controller parameters
Aircraft.Control.P_heading = 6;
