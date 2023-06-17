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
% vehicle = 'super';
% FMU-R version

if strcmpi(vehicle, 'malt')
    fmu_version = "mini";
elseif strcmpi(vehicle, 'super')
    fmu_version = "v2";
elseif strcmpi(vehicle, 'ale')
    fmu_version = "v1";
elseif strcmpi(vehicle, 'session')
    fmu_version = "v2";
else
    ME = MException("UASpaaro:noSuchVehicle", sprintf("No vehicle %s", vehicle));
    throw(ME);
end

% VMS Only flag
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
