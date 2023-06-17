close all;
clear;
clc;

%% Generate truth data for testing
angle_deg_truth = -10;
angle_rad_truth = deg2rad(angle_deg_truth);
dist_to_tar_truth = 7;

target_point = [25 5];

start_point = [15 5];

wp_line = target_point - start_point;

wp_line_len = norm(wp_line);

wp_line_bearing = atan2(wp_line(2),wp_line(1));
if wp_line_bearing < 0
    wp_line_bearing = wp_line_bearing + 2 * pi;
end

tar_to_cur_bearing = wp_line_bearing + angle_rad_truth;

rot_mat = [cos(tar_to_cur_bearing) -sin(tar_to_cur_bearing);
          sin(tar_to_cur_bearing) cos(tar_to_cur_bearing)]; 

tar_to_cur_vec = rot_mat * [1; 0] * dist_to_tar_truth;

cur_point = target_point - tar_to_cur_vec';

plot ([start_point(2), target_point(2)], [start_point(1), target_point(1)], ...
    'DisplayName','WP line')
hold on 
plot (start_point(2), start_point(1),'*',...
    'DisplayName','start')
plot ([cur_point(2), target_point(2)], [cur_point(1), target_point(1)],'--',...
    'DisplayName','cur')
legend
xlim([0 30])
ylim([0 30])
grid on
grid minor
axis equal
xlabel ('Y')
ylabel ('X')

%% Deviation code
bearing_start_to_tar = atan2(target_point(2)-start_point(2),target_point(1)-start_point(1));
deviation_asix_bearing = bearing_start_to_tar + pi/2;
unit_deviation_vec = [cos(deviation_asix_bearing), sin(deviation_asix_bearing)];

vec_tar_to_start = [target_point(1) - start_point(1), target_point(2) - start_point(2)];

vec_tar_to_cur = [target_point(1) - cur_point(1), target_point(2) - cur_point(2)];

len_tar_to_start = norm(vec_tar_to_start);

len_tar_to_cur = norm(vec_tar_to_cur);

y = vec_tar_to_cur(2) * vec_tar_to_start(1) - vec_tar_to_cur(1) * vec_tar_to_start(2);
x = vec_tar_to_start(1) * vec_tar_to_cur(1) + vec_tar_to_start(2)*vec_tar_to_cur(2);

sign_angle =atan2(y,x) ;
sign_ang_deg = rad2deg(sign_angle);

deviation = y * len_tar_to_cur / sqrt(x^2 + y^2);
deviation_vec = deviation * unit_deviation_vec;




