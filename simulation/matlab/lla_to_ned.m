function ned_pos = lla_to_ned(pos_lla, ref_lla)
% lat, lon, alt; lat/lon in degrees, alt in m

c_lat = cosd(ref_lla(1));
s_lat = sind(ref_lla(1));
c_lon = cosd(ref_lla(2));
s_lon = sind(ref_lla(2));
R = [-s_lat * c_lon, -s_lon, -c_lat * c_lon; ...
     -s_lat * s_lon, c_lon, -c_lat * s_lon; ...
     c_lat, 0, -s_lat];
 
ref_E = lla_to_ECEF(ref_lla);
pos_E = lla_to_ECEF(pos_lla);
ned_pos = single(R' * (pos_E - ref_E));


end