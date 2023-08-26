close all
clear
clc

load malt1_109.mat;

pos_hold_log = 1;
wp_log = 1;

nav_init_ind = find(bfs_ins_initialized);
nav_init_ind = nav_init_ind(1);

roll_ang_cmd = vms_aux3;
roll_rate_cmd = vms_aux0;
pitch_ang_cmd = vms_aux4;
pitch_rate_cmd = vms_aux1;
heading_cmd = vms_aux5;
yaw_rate_cmd = vms_aux2;

vx_cmd = vms_aux17;
pos_x_cmd = vms_aux16;
vy_cmd = vms_aux19;
pos_y_cmd = vms_aux18;
vz_cmd = vms_aux15;
pos_z_cmd = vms_aux14;

ax_cmd = vms_aux14;
ay_cmd = vms_aux15;
az_cmd = vms_aux16;

filtered_az = lowpass(fmu_imu_accel_z_mps2, 1, 200);

figure()
subplot(3,1,1)
plot (sys_time_s, rad2deg(roll_rate_cmd),'DisplayName','roll rate cmd')
hold on
plot (sys_time_s, rad2deg(bfs_ins_gyro_x_radps), 'DisplayName','roll rate')
grid on
grid minor
legend
subplot(3,1,2)
plot (sys_time_s, rad2deg(pitch_rate_cmd),'DisplayName','pitch rate cmd')
hold on
plot (sys_time_s, rad2deg(bfs_ins_gyro_y_radps), 'DisplayName','pitch rate')
grid on
grid minor
legend
subplot(3,1,3)
plot (sys_time_s, rad2deg(yaw_rate_cmd),'DisplayName','yaw rate cmd')
hold on
plot (sys_time_s, rad2deg(bfs_ins_gyro_z_radps), 'DisplayName','yaw rate')
grid on
grid minor
legend

figure()
subplot(3,1,1)
plot (sys_time_s, rad2deg(roll_ang_cmd),'DisplayName','roll cmd')
hold on
plot (sys_time_s, rad2deg(bfs_ins_roll_rad), 'DisplayName','roll')
grid on
grid minor
legend
subplot(3,1,2)
plot (sys_time_s, rad2deg(pitch_ang_cmd),'DisplayName','pitch cmd')
hold on
plot (sys_time_s, rad2deg(bfs_ins_pitch_rad), 'DisplayName','pitch')
grid on
grid minor
legend
subplot(3,1,3)
plot (sys_time_s, rad2deg(heading_cmd),'DisplayName','heading cmd')
hold on
plot (sys_time_s, rad2deg(bfs_ins_heading_rad) , 'DisplayName','heading')
grid on
grid minor
legend

if (pos_hold_log == 1 || wp_log == 1)
figure()
subplot(3,1,1)
plot (sys_time_s, vx_cmd,'DisplayName','ve x cmd')
hold on
plot (sys_time_s, bfs_ins_north_vel_mps, 'DisplayName','ve x')
grid on
grid minor
legend
subplot(3,1,2)
plot (sys_time_s, vy_cmd,'DisplayName','ve y cmd')
hold on
plot (sys_time_s, bfs_ins_east_vel_mps, 'DisplayName','ve y')
grid on
grid minor
legend
subplot(3,1,3)
plot (sys_time_s, -vz_cmd,'DisplayName','ve z cmd')
hold on
plot (sys_time_s, -bfs_ins_down_vel_mps, 'DisplayName','ve z')
plot (sys_time_s, -vms_aux23,'DisplayName','filtered ve z');
grid on
grid minor
legend

figure(4)
subplot(3,1,1)
plot (sys_time_s, pos_x_cmd,'DisplayName','tar x')
hold on
plot (sys_time_s, aux_ins_ned_pos_north_m, 'DisplayName','x')
grid on
grid minor
legend
subplot(3,1,2)
plot (sys_time_s, pos_y_cmd,'DisplayName','tar y')
hold on
plot (sys_time_s, aux_ins_ned_pos_east_m, 'DisplayName','y')
grid on
grid minor
legend
subplot(3,1,3)
plot (sys_time_s, -pos_z_cmd,'DisplayName','tar z')
hold on
plot (sys_time_s, bfs_ins_alt_wgs84_m, 'DisplayName','z')
plot (sys_time_s(1000:end), ext_gnss1_alt_wgs84_m(1000:end),'DisplayName', 'gps z')
%plot (sys_time_s, opflow_range_mm/1000);
%ylim([-1 4])
grid on
grid minor
legend
end

if (wp_log == 1)
cage_origin = [33.2154770, -87.5436600, 0];
wp = [1 1 .5;
      4 1 .5;
      4 2 .5;
      1 2 .5];
start_ind = 1000;
lla = [rad2deg(bfs_ins_lat_rad(start_ind:end)), rad2deg(bfs_ins_lon_rad(start_ind:end)), bfs_ins_alt_wgs84_m(start_ind:end)];
cage_pos = lla2ned(lla, cage_origin,'flat');
gps_lla = [rad2deg(ext_gnss1_lat_rad(start_ind:end)), rad2deg(ext_gnss1_lon_rad(start_ind:end)), ext_gnss1_alt_wgs84_m(start_ind:end)];
gps_pos = lla2ned (gps_lla, cage_origin,'flat');
figure(5)
%plot3(cage_pos(:,1),-cage_pos(:,2), -cage_pos(:,3))
%hold on
plot3(gps_pos(:,1),-gps_pos(:,2), -gps_pos(:,3),'.')
hold on
plot3 ([0;8],[0;4],[0;5],'*')
grid on
grid minor
axis equal
end

hor_err = ((pos_x_cmd - aux_ins_ned_pos_north_m).^2 + (pos_y_cmd - aux_ins_ned_pos_east_m).^2).^0.5;
ver_err = abs(pos_z_cmd + bfs_ins_alt_wgs84_m);
%ver_err = abs(vms_aux19 - aux_ins_ned_pos_down_m);
%hor_err = ((vms_aux17 - aux_ins_ned_pos_north_m).^2 + (vms_aux18 - aux_ins_ned_pos_east_m).^2).^0.5;
tot_err = (hor_err.^2 + ver_err.^2).^0.5;
figure()
subplot (3,1,1)
plot (sys_time_s, hor_err,'DisplayName','hor')
grid on
grid minor
legend
subplot(3,1,2)
plot (sys_time_s, ver_err,'DisplayName','ver')
grid on
grid minor
legend
subplot (3,1,3)
plot (sys_time_s, tot_err,'DisplayName','tot')
grid on
grid minor
legend

figure()
plot (sys_time_s, vms_pwm_cmd0)
hold on
plot (sys_time_s, vms_pwm_cmd1)
plot (sys_time_s, vms_pwm_cmd2)
plot (sys_time_s, vms_pwm_cmd3)
hold off
legend

figure()
plot (sys_time_s, fmu_imu_accel_x_mps2);
hold on
plot (sys_time_s, fmu_imu_accel_y_mps2);
plot (sys_time_s, fmu_imu_accel_z_mps2);

figure()
plot (sys_time_s, pos_z_cmd + bfs_ins_alt_wgs84_m)
hold on
plot (sys_time_s, vz_cmd)
plot (sys_time_s, bfs_ins_down_vel_mps)
legend