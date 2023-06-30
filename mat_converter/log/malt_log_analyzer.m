close all
clear
clc

%load malt1_179.mat;
load malt1_186.mat;

figure(1)
subplot(3,1,1)
plot (sys_time_s, rad2deg(vms_aux0),'DisplayName','roll rate cmd')
hold on
plot (sys_time_s, rad2deg(bfs_ins_gyro_x_radps), 'DisplayName','roll rate')
grid on
grid minor
legend
subplot(3,1,2)
plot (sys_time_s, rad2deg(vms_aux1),'DisplayName','pitch rate cmd')
hold on
plot (sys_time_s, rad2deg(bfs_ins_gyro_y_radps), 'DisplayName','pitch rate')
grid on
grid minor
legend
subplot(3,1,3)
plot (sys_time_s, rad2deg(vms_aux2),'DisplayName','yaw rate cmd')
hold on
plot (sys_time_s, rad2deg(bfs_ins_gyro_z_radps), 'DisplayName','yaw rate')
grid on
grid minor
legend

figure(2)
subplot(3,1,1)
plot (sys_time_s, rad2deg(vms_aux3),'DisplayName','roll cmd')
hold on
plot (sys_time_s, rad2deg(bfs_ins_roll_rad), 'DisplayName','roll')
plot (sys_time_s, vms_mode * 10)
grid on
grid minor
legend
subplot(3,1,2)
plot (sys_time_s, rad2deg(vms_aux4),'DisplayName','pitch cmd')
hold on
plot (sys_time_s, rad2deg(bfs_ins_pitch_rad), 'DisplayName','pitch')
grid on
grid minor
legend
subplot(3,1,3)
plot (sys_time_s, rad2deg(vms_aux5),'DisplayName','heading cmd')
hold on
plot (sys_time_s, rad2deg(bfs_ins_heading_rad) , 'DisplayName','heading')
grid on
grid minor
legend

figure(3)
subplot(3,1,1)
plot (sys_time_s, vms_aux6,'DisplayName','vb x cmd')
hold on
plot (sys_time_s, bfs_ins_north_vel_mps, 'DisplayName','vb x')
grid on
grid minor
legend
subplot(3,1,2)
plot (sys_time_s, vms_aux8,'DisplayName','vb y cmd')
hold on
plot (sys_time_s, bfs_ins_east_vel_mps, 'DisplayName','vb y')
grid on
grid minor
legend
subplot(3,1,3)
plot (sys_time_s, -vms_aux10,'DisplayName','vz cmd')
hold on
plot (sys_time_s, -bfs_ins_down_vel_mps, 'DisplayName','vz')
grid on
grid minor
legend

figure(4)
subplot(3,1,1)
plot (sys_time_s, vms_aux15,'DisplayName','tar x')
hold on
plot (sys_time_s, aux_ins_ned_pos_north_m, 'DisplayName','x')
grid on
grid minor
legend
subplot(3,1,2)
plot (sys_time_s, vms_aux16,'DisplayName','tar y')
hold on
plot (sys_time_s, aux_ins_ned_pos_east_m, 'DisplayName','y')
grid on
grid minor
legend
subplot(3,1,3)
plot (sys_time_s, -vms_aux12,'DisplayName','tar z')
hold on
plot (sys_time_s, -vms_aux13, 'DisplayName','z')
plot (sys_time_s, ext_gnss1_alt_wgs84_m,'DisplayName', 'gps z')
grid on
grid minor
legend

pos_err = [vms_aux11 - aux_ins_ned_pos_north_m, vms_aux12 - aux_ins_ned_pos_east_m,...
    vms_aux13 - aux_ins_ned_pos_down_m];
dis_err = (pos_err(:,1).^2 + pos_err(:,2).^2 + pos_err(:,3).^2).^0.5;

cage_origin = [33.2154770, -87.5436600, 0];
wp = [1 1 .5;
      4 1 .5;
      4 2 .5;
      1 2 .5];
start_ind = 1;
lla = [rad2deg(bfs_ins_lat_rad(start_ind:end)), rad2deg(bfs_ins_lon_rad(start_ind:end)), bfs_ins_alt_wgs84_m(start_ind:end)];
cage_pos = lla2ned(lla, cage_origin,'flat');
gps_lla = [rad2deg(ext_gnss1_lat_rad(start_ind:end)), rad2deg(ext_gnss1_lon_rad(start_ind:end)), ext_gnss1_alt_wgs84_m(start_ind:end)];
gps_pos = lla2ned (gps_lla, cage_origin,'flat');
figure(5)
plot3(cage_pos(:,1),-cage_pos(:,2), -cage_pos(:,3))
hold on
plot3(gps_pos(:,1),-gps_pos(:,2), -gps_pos(:,3),'.')
plot3 ([0;8],[0;4],[0;5],'*')
grid on
grid minor
axis equal

figure()
plot (sys_time_s, -(vms_aux14.^2 + vms_aux15.^2 + vms_aux16.^2).^0.5 + 1)
hold on
plot (sys_time_s, bfs_ins_accel_z_mps2/9.81 + 1)



hor_err = ((vms_aux15 - aux_ins_ned_pos_north_m).^2 + (vms_aux16 - aux_ins_ned_pos_east_m).^2).^0.5;
ver_err = abs(vms_aux14 - aux_ins_ned_pos_down_m);
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
plot (sys_time_s, vms_pwm_cmd0);
hold on
plot (sys_time_s, vms_pwm_cmd1);
plot (sys_time_s, vms_pwm_cmd2);
plot (sys_time_s, vms_pwm_cmd3);
plot (sys_time_s, incept_ch6);

figure()
plot (sys_time_s, vms_throttle_cmd_prcnt/100);
