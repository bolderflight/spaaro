function ecef_pos = lla_to_ECEF(lla)
% lat, lon, alt; lat/lon in degrees, alt in m

FLATTENING = 1 / 298.257223563;
ECCENTRICITY = sqrt(FLATTENING * (2 - FLATTENING));

re = calc_ew_rad(lla(1));
c_lat = cosd(lla(1));
s_lat = sind(lla(1));
c_lon = cosd(lla(2));
s_lon = sind(lla(2));

x = (re + lla(3)) * c_lat * c_lon;
y = (re + lla(3)) * c_lat * s_lon;
z = ((1 - ECCENTRICITY^2) * re + lla(3)) * s_lat;

ecef_pos = [x; y; z];

end

function ew_rad = calc_ew_rad(lat)
FLATTENING = 1 / 298.257223563;
ECCENTRICITY = sqrt(FLATTENING * (2 - FLATTENING));
EQ_RAD = 6378137;

ew_rad = EQ_RAD / sqrt(1 - ECCENTRICITY^2 * sind(lat)^2);
end