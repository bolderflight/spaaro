%% Plot the result of simulation
file_name = '20220208-1102_simout.mat';
full_name = strcat(pwd,'\simout\', file_name);
load (full_name);

start_ind = 10;

time_s = fi2double(Data.sys_time_us / 10^6);
time_s = time_s (start_ind:end);

references = [Data.waypoint_x

figure('Name','Altitude tracking')
plot (time_s,Data.waypoint_z(start_ind:end),time_s,Data.nav_alt_rel_m(start_ind:end))
grid minor

figure ('Name', 'Lattitudde tracking')
plot (time_s,Data.waypoint_x(start_ind:end) ./(10^7),time_s,Data.nav_lat_rad(start_ind:end) .* 180/pi)
grid minor

figure ('Name', 'Longtitude tracking')
plot (time_s,Data.waypoint_y (start_ind:end)./(10^7),time_s,Data.nav_lon_rad(start_ind:end) .* 180/pi)
grid minor
