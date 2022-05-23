function ned_pos = mission_item_to_ned(x, y, z, home_lat_rad, home_lon_rad, home_alt)
% ind assumes flight_plan is 0 based index but matlab is 1 based
r2d = 180 / pi;

lat = single(x) * 0.0000001;
lon = single(y) * 0.0000001;
alt = single(z + home_alt);

ned_pos = single(lla_to_ned([lat, lon, alt], ...
                            [home_lat_rad * r2d, home_lon_rad * r2d, home_alt]));

end