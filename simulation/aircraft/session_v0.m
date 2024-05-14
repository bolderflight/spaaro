% Initialize a vehicle for use in the simulation
% A Quadplane model (Based on Iteration1 presented in SESSION's PDR)
%
% Aabhash Bhandari


%% Platform's name
Aircraft.name = 'session_v0';

%% Mass properties 
% CG is at the body origin
% Obtained from OpenVSP model SESSION_iter1

% Mass [kg]
Aircraft.Mass.mass_kg = 9.594;
% c.g. location [m]
Aircraft.Mass.cg_m = [0, 0, 0];

% Moments of inertia [kg*m^2] obtained from OpenVSP model 
Aircraft.Mass.ixx_kgm2 = 1.231;
Aircraft.Mass.iyy_kgm2 = 1.399;
Aircraft.Mass.izz_kgm2 = 2.602;
Aircraft.Mass.ixz_kgm2 = 0.00;
Aircraft.Mass.inertia_kgm2 = [Aircraft.Mass.ixx_kgm2    0   -Aircraft.Mass.ixz_kgm2;...
                              0          Aircraft.Mass.iyy_kgm2          0;...
                              -Aircraft.Mass.ixz_kgm2   0       Aircraft.Mass.izz_kgm2];

% Computing matrix inverses
Aircraft.Mass.inertia_inv = inv(Aircraft.Mass.inertia_kgm2);
Aircraft.Mass.inertia_inv_4by4 = inv([[Aircraft.Mass.inertia_kgm2, [0;0;0]]; [0,0,0,1]]);



%% Geometric parameters

% Reference chord, m
Aircraft.Geom.c_m = 0.150;
% Reference span, m
Aircraft.Geom.b_m = 2.75;
% Reference area, m^2
Aircraft.Geom.s_m2 = 0.413;

% Center of pressure [x y z], m
Aircraft.Geom.cp_m = [0.0407 0 0];

%% Effectors
% Number of PWM channels
Aircraft.Eff.nPwm = 8;
% Number of SBUS channels
Aircraft.Eff.nSbus = 16;
% Total number of channels
Aircraft.Eff.nCh = Aircraft.Eff.nPwm + Aircraft.Eff.nSbus;

%% Control surfaces

% Number of control surfaces
% [ail, elev, ruder]
% Positive TED, TEL

Aircraft.Surf.nSurf = 3;
% PWM / SBUS channel number to control surface mapping
% Array ordered as 8 PWM channels then 16 SBUS channels for a total of 24
% channels, 1-based indexing

% 6 = both ailerons, 7 = elevators, 8 = rudders
Aircraft.Surf.map = [6, 7, 8];
% Rate limits
Aircraft.Surf.Limit.rate_dps = 150 * ones(Aircraft.Surf.nSurf, 1);
% Position limits
Aircraft.Surf.Limit.pos_deg = 30 * ones(Aircraft.Surf.nSurf, 1);
Aircraft.Surf.Limit.neg_deg = -30 * ones(Aircraft.Surf.nSurf, 1);

% Servo Actuator first-order dynamics time constant 
% From WVU YF-22 research UAVs 
% https://researchrepository.wvu.edu/cgi/viewcontent.cgi?article=3634&context=etd
Aircraft.Surf.time_constant = 0.0424;



%% Hover Aerodynamics
% Not implemented for now. 

% k where ||Drag|| = k * airspeed
% linear assumption for low speed is good enough
Aircraft.Aero.linear_drag_coef = 0;

% cutoff airspeed
% below this speed, only include Quadcopter aero forces and moments
% above this speed, include full forward flight's aero forces and moments 
Aircraft.Aero.cutoff_airspeed_mps = 5;

%% Weighted Sum between forward and hover aerodynamics

% Using a Sigmoidal MF to calculate weight of aerodynamics contribution
% x = u_mps
% a = shape parameter, c = inflection point (on u_mps)
Aircraft.Aero.sigmoidal_shape = 1.2;
Aircraft.Aero.sigmoidal_inflection = 4.5;   

%% Forward Flight Aerodynamics

% Axis system for aerodynamic coefficients
% https://www.mathworks.com/help/aeroblks/aerodynamicforcesandmoments.html
% 1 = Wind axis
% 2 = Stability axis
% 3 = Body axis

% Wind axis is selected because OpenVSP gives output in this frame and is
% easier to deal with 
Aircraft.Aero.axis = 3;

% Using StabilityCoefAndDerivatives class in utils/, base coefficients and 
% derivatives are used here to model aerodynamic forces and moments. 

% Note: Base values are taken at 0 deg, and derivatives are taken for 4 deg
% point. This is because the derivatives are more consistent after 4 deg,
% and they are assumed to be linear, so the derivatives at 4 deg are
% assumed for 0 deg as well.

% get control derivatives from .stab file output

% Format of the array is:: 
% [base_val, alpha, beta, p, q, r, aileron_def, elevator_def, rudder_def]

% Lift
Aircraft.Aero.CL_coefs = [0.9181, 6.8855, 0, 0, 18.7786, 0, 0, 0.8388, 0];

% Drag
Aircraft.Aero.CD_coefs = [0.06519, 0.5768, 0, 0, -0.7326, 0, 0, 0.1012, 0];

% Side Force
Aircraft.Aero.CY_coefs = [0.0000, 0, -0.4412, -0.0205, -0.003166, -0.1137, 0.1638, 0, -0.2071];

% X-axis moment
Aircraft.Aero.Cl_coefs = [-0.0004, 0, -0.0719, -0.6494, -0.0457, -0.1962, 0.3357, 0, 0.010322];

% Y-axis moment
% ignored Cm_q = -101.6445 (not sure why its that low)
Aircraft.Aero.Cm_coefs = [-0.0447, -1.9881, 0, 0, -101.6445, 0, 0, 4.1454, 0];

% Z-axis_moment
Aircraft.Aero.Cn_coefs = [0.0000, 0, 0.0015, -0.1405, -0.0020, -0.03442, -0.0165, 0, 0.0371];

%% Motor Maps 

% Number of motors
Aircraft.Motor.nMotor = 5;

% Assign a pwm channel to motor
% 1, 2, 3, 4 are hover motors, and 5 is forward motor. 
Aircraft.Motor.map = [ 1 ; 2 ; 3 ; 4; 5];

% Motor bandwidth radps 
Aircraft.Motor.bandwidth = 100;

% Motor positions relative to c.g in [m] [x,y,z](obtained from OpenVSP)
% First 4 Motor numbers and order using Arducopter convention (QUAD-H)
% 5th motor is the forward motor. (pusher configuration at the end of fuselage)

Aircraft.Motor.pos_m = [0.610   0.5    0;...
                        -0.6   -0.5   0;...
                        0.610    -0.5   0;...
                        -0.6   0.5    0;...
                        -0.55   0   0]; 

% NOTE: Use this motor position for debugging hover sim (all equidistant from CG)
% Aircraft.Motor.pos_m = [0.60   0.5    0;...
%                         -0.6   -0.5   0;...
%                         0.60    -0.5   0;...
%                         -0.6   0.5    0;...
%                         -0.55   0   0]; 


% Alignment of thrust with body frame x, y, z axis
% All hover rotors are aligned so that thrust is in -z
% The forward flight rotor has thrust on +x. 
Aircraft.Motor.align = [0, 0, -1;...
                            0, 0, -1;...
                            0, 0, -1;...
                            0, 0, -1;...
                            1, 0, 0];

% Motor rotation direction (right hand rule with z_body and x_body)
% Motor 1 and 2 are cw and motor 3 and 4 are ccw
Aircraft.Motor.dir = [1; 1; -1; -1; 1];

% Use 1 for motors facing the forward direction. 
Aircraft.Motor.forward_bool = [0; 0; 0; 0; 1];
Aircraft.Motor.forward_index = find(Aircraft.Motor.forward_bool == 1);

%% Hover Propulsion System

% Tmotor MN-605s 320 kV, 21x6.3 Prop

% Motor speed constant Kv
Aircraft.HoverMotor.kv = 320; % Kv of T-Motor MN605-S

% Motor zero load current [Amp]
Aircraft.HoverMotor.io = 2.2; 

% Motor internal resistance [Ohm]
Aircraft.HoverMotor.r = 0.021 ;

% Coef of torque of MN1005 motor based on T-Motor's website
Aircraft.HoverMotor.kq = 0.0411;     %N-m/A

% Diameter [inches]
Aircraft.HoverRotor.dia_in = 21;

% Coefficient of thrust constant obtained from T-motor's website data
Aircraft.HoverRotor.kt = 0.0388;   %N-m/N

% Thrust and torque models obtained from Tmotor's data
% for throttle 0-1
% 2nd order polyfit on thrust(N), 0 throttle = 0 thrust
Aircraft.HoverRotor.poly_thrust = [48.8680 16.6778 -0.3264];
% 2nd order polyfit on torque
Aircraft.HoverRotor.poly_torque = [1.1138 0.3510 -0.0204];


%% Forward Flight Propulsion system

% Tmotor AT5220-B, kV 380, APC 19x10 prop

% Speed constant, Kv [RPM/V]
Aircraft.ForwardMotor.kv = 380;
% Resistance [ohm]
Aircraft.ForwardMotor.r = 0.011;
% Zero torque current [Amp]
Aircraft.ForwardMotor.io = 4;

% Diameter [inches]
Aircraft.ForwardProp.dia_in = 19;

% See simulation/utils/get_prop_coefs.m to get polyfits

% Coefficient of thrust polynomial coefficients
Aircraft.ForwardProp.ct = [-0.2594   -0.0480    0.0216];
% Coefficient of power polynomial coefficients
Aircraft.ForwardProp.cp = [-0.0826   -0.0525    0.0066    0.0022];

% Approx. maximum thrust with given configuration (from Tmotor)
Aircraft.ForwardProp.max_thrust_N = 75;

% hand calculated using available hardware
% Electric motor and propeller combine moment of inertia [kg*m^2]
Aircraft.ForwardProp.Jmp_kgm2 = 7.9421e-4;



%% Battery

% Number of battery cells
Aircraft.Battery.nCell = 6;
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
Aircraft.Sensors.Imu.Gyro.bias_radps = 3e-5 .* [1, 1, 1]';
% G-sensitivity in rad/s per m/s/s
Aircraft.Sensors.Imu.Gyro.accel_sens_radps = [0 0 0]';  
Aircraft.Sensors.Imu.Gyro.noise_radps = deg2rad(0.00001) * ones(3, 1);
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
Aircraft.Sensors.PitotStaticInstalled = 1;
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

Aircraft.Control.wp_radius = 0;

% FixedWing Angular Rate INDI controller
Aircraft.Control.Forward.indi_pqr_gain = [8,8,15];
Aircraft.Control.Forward_v2.indi_pqr_gain = [3, 3, 5];

% cutoff frequency for LP filter on inner loop surface deflection outputs
Aircraft.Control.Forward.surf_def_out_LP_filter_CTOFF = 5;

% yaw damper
Aircraft.Control.Forward.yaw_damper_gain = 2.0;

% cutoff frequency for LP filter used for sideslip controller (Hz).
Aircraft.Control.Forward.sideslip_accel_LP_filter_CTOFF = 0.5;

% FixedWing Attitude Linear Controller Gains (Roll-pitch)
Aircraft.Control.Forward.Att_err_gain = [3, 2.5];
% Aircraft.Control.Forward_v2.att_p_gain = [10, 12.5];
Aircraft.Control.Forward_v2.att_p_gain = [3, 2.5];
Aircraft.Control.Forward_v2.att_d_gain = [0, 0.25];

% FixedWing Attitude Linear Controller D gains (Roll-pitch)
Aircraft.Control.Forward.Att_D_gain = [0.5, 0.15];

% FixedWing Attitude Controller pqr max rates
Aircraft.Control.Forward.max_pqr_ref = 1.2;

% FixedWing Attitude Sideslip Controller Gains (PIDs)
Aircraft.Control.Forward.Sideslip_gains = [7 2.5 4];

% FixedWing Total Heading Control System 
Aircraft.Control.Forward.thcs.error_gain = 0.4;
Aircraft.Control.Forward.thcs.cmd_gain = 0.75;
Aircraft.Control.Forward.thcs.beta_dot_ref_limit = 0.05;

% Sorta turning rate (rad/s)
Aircraft.Control.Forward.thcs.psi_dot_ref = 0.4;

% FixedWing Heading Controller Max Roll Angle 
Aircraft.Control.Forward.max_roll_rad = deg2rad(35);

% FixedWing Outer Loop Controller Max pitch Angle
Aircraft.Control.Forward.max_pitch_rad = deg2rad(20);

% FixedWing Outer Loop Controller Max yaw ref rate
Aircraft.Control.Forward.max_yaw_rate = deg2rad(40);

% FixedWing Heading Controller P-gain
Aircraft.Control.Forward.heading_P = 2.25;

% FixedWing Altitude Controller P-gain
Aircraft.Control.Forward.altitude_P = 1;
Aircraft.Control.Forward_v2.altitude_P = 2; 

% FixedWing Outer Model reference shape
Aircraft.Control.Forward_v2.pqr_ref_k1_k2 = [45, 150];
Aircraft.Control.Forward_v2.outer_indi_ref_k1 = [1, 10];
Aircraft.Control.Forward_v2.outer_indi_ref_k2 = [20, 30];

% FixedWing Outer Loop indi gain (airspeed and flight path control)
Aircraft.Control.Forward.outer_indi_gains = [1.0, 1.5];
Aircraft.Control.Forward_v2.outer_indi_gains = [1, 2.5];

% LP filter on throttle_cmd_out and pitch_ref
Aircraft.Control.Forward.outer_indi_outputs_LP_filter_CTOFF = [0.15, 0.75];
Aircraft.Control.Forward_v2.outer_indi_outputs_LP_filter_CTOFF = [1, 0.75];
Aircraft.Control.Forward_v2.airspeed_ref_LP_filter_CTOFF = 0.075;

% Hover Inner Loop low-pass filters 
% cutoff throttle output
Aircraft.Control.Hover.throttle_output_LP_filter_CTOFF = 1;
% cutoff w and pqr references
Aircraft.Control.Hover.inner_ref_LP_filter_CTOFF = 0.1;

% Hover Inner Loop model references
Aircraft.Control.Hover_v2.pqr_ref_k1_k2 = [35, 50];
Aircraft.Control.Hover_v2.r_ref_k1_k2 = [2, 10];
Aircraft.Control.Hover_v2.w_ref_k1_k2 = [15, 75];

% Hover Inner Loop Body Rates (pqrw) Gain
Aircraft.Control.Hover.body_rates_gain = [1.5, 1.5, 1, 3];
Aircraft.Control.Hover_v2.pq_p_gain = 1.5;
Aircraft.Control.Hover_v2.r_p_gain = 0.5;
Aircraft.Control.Hover_v2.w_p_gain = 8;
Aircraft.Control.Hover_v2.w_i_gain = 10;

% Hover Attitude Control pqr output limits
Aircraft.Control.Hover.pqr_limit = 2;

% Hover Inner Loop Attitude Control Gain (roll, pitch)
Aircraft.Control.Hover.att_gain = 2.25;
Aircraft.Control.Hover_v2.att_gain = 7.5;

% Hover Altitude Control
Aircraft.Control.Hover.alt_gain = 0.75;
Aircraft.Control.Hover_v2.alt_gain = 2.25;

% Hover Inertial and Body vertical speed limits 
Aircraft.Control.Hover.nav_vert_limits = [2, -4];
Aircraft.Control.Hover.body_vert_limits = [4, -6];

% U_cmd will be calculated if airspeed > threshold. else, u_cmd = 0.
Aircraft.Control.Hover.uv_ref_airspeed_threshold = 1;

% Hover Speed control LP filter cutoff
Aircraft.Control.Hover.u_ref_LP_filter_CTOFF = 0.05; 
Aircraft.Control.Hover.roll_pitch_ref_LP_filter_CTOFF = 0.01;
Aircraft.Control.Hover_v2.roll_pitch_ref_LP_filter_CTOFF = 0.1;


% Hover Outer Loop model reference
Aircraft.Control.Hover_v2.uv_ref_k1_k2 = [2, 8];

% Hover Speed Control Gains
Aircraft.Control.Hover.uv_gain = [1.1, 0.2]; 
Aircraft.Control.Hover_v2.uv_p_gain = 1.8;
Aircraft.Control.Hover.uv_d_gain = [2.0, 1.75];

% Roll pitch reference limits
Aircraft.Control.Hover.roll_pitch_ref_limits = [0.25, 0.25];

% Hover heading control 
Aircraft.Control.Hover.heading_gain = 7;
Aircraft.Control.Hover_v2.heading_gain = 1.5;
Aircraft.Control.Hover.yaw_rate_ref_limit = 1;
Aircraft.Control.Hover_v2.yaw_rate_ref_limit = 0.45;

% Mode Switching and Transition related parameters
Aircraft.Control.modes.mode_shutoff_airspeeds = [7, 18];
Aircraft.Control.modes_v2.throttle_min_speed = 5;
Aircraft.Control.modes_v2.max_hover_attitude_tracking_speed = 6;
Aircraft.Control.modes_v2.min_forward_attitude_tracking_speed = 8;
Aircraft.Control.modes_v2.transition_shutoff_airspeed = [6, 17];
Aircraft.Control.modes_v2.detransition_shutoff_airspeed = [8, 18];


Aircraft.Control.modes.hover2forward_airspeed_ramp = 3;
Aircraft.Control.modes.forward2hover_airspeed_ramp = -1.25;

% Parameters of the controller input blending sigmoidal function
Aircraft.Control.modes.transition_sigmoidal_a = 0.4;
Aircraft.Control.modes.transition_sigmoidal_c = 14;

Aircraft.Control.modes_v2.transition_sigmoidal_a = 0.4;
Aircraft.Control.modes_v2.transition_sigmoidal_c = 14;



%% Aircraft Parameters used in Controller
% For nominal case, this will be equal to the expected/known parameters.
% But, these values can be changed to represent aircraft model's parameters
% uncertainties

% Aerodynamic coefficients Coefficients

Aircraft.Ucertain.Forward.CL_coefs = Aircraft.Aero.CL_coefs;
Aircraft.Ucertain.Forward.CD_coefs = Aircraft.Aero.CD_coefs;
Aircraft.Ucertain.Forward.CY_coefs = Aircraft.Aero.CY_coefs;
Aircraft.Ucertain.Forward.Cl_coefs = Aircraft.Aero.Cl_coefs;
Aircraft.Ucertain.Forward.Cm_coefs = Aircraft.Aero.Cm_coefs;
Aircraft.Ucertain.Forward.Cn_coefs = Aircraft.Aero.Cn_coefs;

% Moment of inertia
Aircraft.Ucertain.inertia = Aircraft.Mass.inertia_kgm2;

% Thrust and Torque Curves
Aircraft.Ucertain.hover_thrust_poly_fit = Aircraft.HoverRotor.poly_thrust;
Aircraft.Ucertain.hover_torque_poly_fit = Aircraft.HoverRotor.poly_torque;

% Motor and surface deflections bandwidth
Aircraft.Ucertain.motor_bandwidth = Aircraft.Motor.bandwidth;
Aircraft.Ucertain.surf_time_cons = Aircraft.Surf.time_constant;

% Deviation in the aerodynamics coefficient bases and derivatives
% changing CL0, CL_alpha, Cm_alpha, Cl_p, Cn_p
if AddEffects.aero_moments.sys_dynamics_bias_bool
    Aircraft.Ucertain.Forward.CL_coefs(1) = 0.95 * Aircraft.Ucertain.Forward.CL_coefs(1);
    Aircraft.Ucertain.Forward.CL_coefs(2) = 0.9 * Aircraft.Ucertain.Forward.CL_coefs(2);
    Aircraft.Ucertain.Forward.CD_coefs(1) = 1.25 * Aircraft.Ucertain.Forward.CD_coefs(1);
    Aircraft.Ucertain.Forward.Cm_coefs(2) = 1.2 * Aircraft.Ucertain.Forward.Cm_coefs(2);
    Aircraft.Ucertain.Forward.Cl_coefs(4) = 0.9 * Aircraft.Ucertain.Forward.Cl_coefs(4);
    Aircraft.Ucertain.Forward.CY_coefs(3) = 1.05 * Aircraft.Ucertain.Forward.CY_coefs(3);
end

% Loss of control effectiveness
if AddEffects.aero_moments.loss_control_eff_bias_bool
    Aircraft.Ucertain.Forward.Cl_coefs(7) = 1.15 * Aircraft.Ucertain.Forward.Cl_coefs(7);
    Aircraft.Ucertain.Forward.Cm_coefs(8) = 0.9 * Aircraft.Ucertain.Forward.Cm_coefs(8);
    Aircraft.Ucertain.Forward.Cn_coefs(9) = 0.95 * Aircraft.Ucertain.Forward.Cn_coefs(9);
end


% Deviation in moment of inertia properties (increased by 10%)
if AddEffects.inertia.bias_bool
    Aircraft.Ucertain.inertia = 1.2 .* Aircraft.Ucertain.inertia;
end

% Deviation in actuator model
if AddEffects.motor_bias_bool
    Aircraft.Ucertain.motor_bandwidth = 0.8 * Aircraft.Ucertain.motor_bandwidth;
end
if AddEffects.surf_bias_bool
    Aircraft.Ucertain.surf_time_cons = 1.2 * Aircraft.Ucertain.surf_time_cons;
end

% Actuator faults
if AddEffects.actuator_fault_bool
    
end


% Deviation in thrust and torque coefficients (reduced by 15%)
Aircraft.Ucertain.hover_thrust_poly_fit(1) = 0.85 * Aircraft.Ucertain.hover_thrust_poly_fit(1);
Aircraft.Ucertain.hover_torque_poly_fit(1) = 0.85 * Aircraft.Ucertain.hover_torque_poly_fit(1);



%% Aircraft Specific Initial Conditions

InitCond.motor_cmd = [0.4, 0.4, 0.4, 0.4, 0];
InitCond.surface_rad = [0 0 0];
% 
% InitCond.motor_cmd = [0, 0, 0, 0, 0.5];
% InitCond.surface_rad = [0 0 0];
%
% Forward prop rotation rate (rad/s)
InitCond.engine_speed_radps = 4000 * (2*pi/60);

%% SESSION specific simulation settings

% sample time for fixed-step solver 
SimConfig.sample_time = 0.01;
