close all;
clear;
clc;

load malt1_13.mat

raw_gyro = [fmu_imu_gyro_x_radps, fmu_imu_gyro_y_radps,fmu_imu_gyro_z_radps];
raw_acc = [fmu_imu_accel_x_mps2, fmu_imu_accel_y_mps2, fmu_imu_accel_z_mps2];

filtered_gyro = [bfs_ins_gyro_x_radps, bfs_ins_gyro_y_radps, bfs_ins_gyro_z_radps];
filtered_acc = [bfs_ins_accel_x_mps2, bfs_ins_accel_y_mps2, bfs_ins_accel_z_mps2];
% pitch_cc = vms_aux(:,6);
% roll_cc = vms_aux(:,2);
% yaw_cc = vms_aux(:,10);
% 
% controller_output = [roll_cc, pitch_cc, yaw_cc];

%% Second order LPF and IIR filter comparison
f_cutoff = 30;
lpf2p_data = raw_gyro;
iir_data = raw_gyro;
acc_lpf2p_data = raw_gyro(:,1);

gyro_x_LPF2p = LPF2p;
gyro_y_LPF2p = LPF2p;
gyro_z_LPF2p = LPF2p;

acc_z_LPF2P = LPF2p;

gyro_x_LPF2p = gyro_x_LPF2p.init_filter(200, f_cutoff);
gyro_y_LPF2p = gyro_y_LPF2p.init_filter(200, f_cutoff);
gyro_z_LPF2p = gyro_z_LPF2p.init_filter(200, f_cutoff);

acc_z_LPF2P = acc_z_LPF2P.init_filter(200,1);
acc_z_LPF2P = acc_z_LPF2P.reset_val(fmu_imu_accel_z_mps2(1));

gyro_x_LPF2p = gyro_x_LPF2p.reset_val(fmu_imu_gyro_x_radps(1));
gyro_y_LPF2p = gyro_y_LPF2p.reset_val(fmu_imu_gyro_y_radps(1));
gyro_z_LPF2p = gyro_z_LPF2p.reset_val(fmu_imu_gyro_z_radps(1));

lpf2p_data(1,:) = [gyro_x_LPF2p.output, gyro_y_LPF2p.output, gyro_z_LPF2p.output];
acc_lpf2p_data(1,1) = acc_z_LPF2P.output;
for i = 2:1:size(lpf2p_data,1)
    gyro_x_LPF2p = gyro_x_LPF2p.apply_filter(raw_gyro(i,1));
    gyro_y_LPF2p = gyro_y_LPF2p.apply_filter(raw_gyro(i,2));
    gyro_z_LPF2p = gyro_z_LPF2p.apply_filter(raw_gyro(i,3));
    acc_z_LPF2P = acc_z_LPF2P.apply_filter(raw_acc(i,3));
    acc_lpf2p_data(i,1) = acc_z_LPF2P.output;
    lpf2p_data(i,:) = [gyro_x_LPF2p.output, gyro_y_LPF2p.output, gyro_z_LPF2p.output];
end

figure()
plot (sys_time_s, -acc_lpf2p_data - 9.81);
hold on 
plot (sys_time_s, (vms_throttle_cmd_prcnt/100) - 0.9);
grid on
grid minor
ylim([-1 1])


