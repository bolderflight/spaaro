% Configures the simulation
%
% Brian R Taylor
% brian.taylor@bolderflight.com
% 
% Copyright (c) 2021 Bolder Flight Systems
%

%% Definitions
% Vehicle
vehicle = 'ultra_stick_25e';
% FMU-R version
fmu_version = "v2";

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

%% Flight software frame rate
if strcmp(upper(fmu_version), "V2")
    frameRate_hz = 100;
else
    frameRate_hz = 50;
end
framePeriod_s = 1/frameRate_hz;
