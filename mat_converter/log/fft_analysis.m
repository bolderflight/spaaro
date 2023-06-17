close all;
clear;
clc;

load malt0.mat

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

gyro_x_LPF2p = LPF2p;
gyro_y_LPF2p = LPF2p;
gyro_z_LPF2p = LPF2p;

gyro_x_IIR = bfs_lpf;
gyro_y_IIR = bfs_lpf;
gyro_z_IIR = bfs_lpf;

gyro_x_LPF2p = gyro_x_LPF2p.init_filter(200, f_cutoff);
gyro_y_LPF2p = gyro_y_LPF2p.init_filter(200, f_cutoff);
gyro_z_LPF2p = gyro_z_LPF2p.init_filter(200, f_cutoff);


gyro_x_LPF2p = gyro_x_LPF2p.reset_val(fmu_imu_gyro_x_radps(1));
gyro_y_LPF2p = gyro_y_LPF2p.reset_val(fmu_imu_gyro_y_radps(1));
gyro_z_LPF2p = gyro_z_LPF2p.reset_val(fmu_imu_gyro_z_radps(1));

lpf2p_data(1,:) = [gyro_x_LPF2p.output, gyro_y_LPF2p.output, gyro_z_LPF2p.output];
for i = 2:1:size(lpf2p_data,1)
    gyro_x_LPF2p = gyro_x_LPF2p.apply_filter(raw_gyro(i,1));
    gyro_y_LPF2p = gyro_y_LPF2p.apply_filter(raw_gyro(i,2));
    gyro_z_LPF2p = gyro_z_LPF2p.apply_filter(raw_gyro(i,3));
    lpf2p_data(i,:) = [gyro_x_LPF2p.output, gyro_y_LPF2p.output, gyro_z_LPF2p.output];
end

F_s = 200;

raw_fft = fftshift(fft(raw_gyro),1);
raw_fft_amp = abs(raw_fft);
filtered_fft = fftshift(fft(filtered_gyro),1);
filtered_fft_amp = abs(filtered_fft);
% controller_fft = fftshift(fft(controller_output),1);
% controller_amp = abs(controller_fft);
raw_accel_fft = fftshift(fft(raw_acc),1);
raw_accel_amp = abs(raw_accel_fft);
filtered_accel_fft = fftshift(fft(filtered_acc),1);
fitlered_accel_amp = abs(filtered_accel_fft);
lpf2p_fft = fftshift(fft(lpf2p_data),1);
lpf2p_amp = abs(lpf2p_fft);

freq_vec = linspace(-F_s/2, F_s/2, size(raw_fft,1));
figure(1)
plot (freq_vec,raw_fft_amp(:,1), 'DisplayName','raw')
hold on 
plot (freq_vec,filtered_fft_amp(:,1), 'DisplayName','filterd')
%plot (freq_vec,lpf2p_amp(:,1), 'DisplayName','lpf2p')
grid on
grid minor
legend
xlim([0 100])
title ('Roll FFT')

figure(2)
plot (freq_vec,raw_fft_amp(:,2), 'DisplayName','raw')
hold on 
plot (freq_vec,filtered_fft_amp(:,2), 'DisplayName','filterd')
%plot (freq_vec,lpf2p_amp(:,2), 'DisplayName','lpf2p')
grid on
grid minor
legend
xlim([0 100])
title ('Pitch FFT')

figure(3)
plot (freq_vec,raw_fft_amp(:,3), 'DisplayName','raw')
hold on 
plot (freq_vec,filtered_fft_amp(:,3), 'DisplayName','filterd')
%plot (freq_vec,lpf2p_amp(:,3), 'DisplayName','lpf2p')
grid on
grid minor
legend
xlim([0 100])
title ('Yaw FFT')

figure (4)
plot (freq_vec, raw_accel_amp(:,1))
hold on
plot (freq_vec, raw_accel_amp(:,2)) 
plot (freq_vec, raw_accel_amp(:,3))
legend ('x', 'y','z')
grid on
grid minor
xlim([0 100])
ylim([0 20000])

figure (5)
plot (freq_vec, fitlered_accel_amp(:,1))
hold on
plot (freq_vec, fitlered_accel_amp(:,2)) 
plot (freq_vec, fitlered_accel_amp(:,3))
legend ('x', 'y','z')
grid on
grid minor
xlim([0 100])
ylim([0 20000])

figure(6)
plot (sys_time_s, fmu_imu_gyro_x_radps, 'DisplayName','raw')
hold on
plot (sys_time_s, lpf2p_data(:,1),'DisplayName','LPF2p')
plot (sys_time_s, iir_data(:,1), 'DisplayName', 'IIR')
legend
grid on
grid minor

figure(7)
plot (sys_time_s, fmu_imu_gyro_y_radps, 'DisplayName','raw')
hold on
plot (sys_time_s, lpf2p_data(:,2),'DisplayName','LPF2p')
plot (sys_time_s, iir_data(:,2), 'DisplayName', 'IIR')
legend
grid on
grid minor

figure(8)
plot (sys_time_s, fmu_imu_gyro_z_radps, 'DisplayName','raw')
hold on
plot (sys_time_s, lpf2p_data(:,3),'DisplayName','LPF2p')
plot (sys_time_s, iir_data(:,3), 'DisplayName', 'IIR')
legend
grid on
grid minor

figure(9)
plot (sys_time_s, bfs_ins_accel_x_mps2,'DisplayName','x');
hold on
plot (sys_time_s, bfs_ins_accel_y_mps2,'DisplayName','y')
plot (sys_time_s, bfs_ins_accel_z_mps2,'DisplayName','z')
legend
grid on
grid minor

