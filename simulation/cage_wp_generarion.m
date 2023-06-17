close all;
clear;
clc;

cage_wp = [2, 2, 2;
           5, 2, 2;
           5, 4, 2;
           2, 4, 2;
           2, 2, 2];

cage_origin_lla_deg_m = [33.2154770, -87.5436600, 0];

%% Test cage plot
figure(1)
plot (cage_wp(:,1), cage_wp(:,2))
hold on
plot (cage_wp(:,1), cage_wp(:,2),'*')
xlim([0 8])
ylim([0 5])

cage_wp(:,2) = -cage_wp(:,2);
coor_wp_lla_deg_m = ned2lla (cage_wp, cage_origin_lla_deg_m,'flat');
% Flip z pos to alt
coor_wp_lla_deg_m (:,3) = - coor_wp_lla_deg_m(:,3);

wp_file_matrix = zeros(size(cage_wp,1)+1,12);
wp_file_matrix(:,1) =  0:size(cage_wp,1);
wp_file_matrix(1,2) = 1;
wp_file_matrix(:,4) = 16;
wp_file_matrix(2:end,3) = 3;
wp_file_matrix(1,9:11) = cage_origin_lla_deg_m;
wp_file_matrix(1:end-1,end) = 1;
wp_file_matrix(2:end, 9:11) = coor_wp_lla_deg_m;

file_name = 'cage_wp.txt';
fid = fopen(file_name,'w');
fprintf(fid,'QGC WPL 110');
writematrix (wp_file_matrix,file_name,'Delimiter','tab','WriteMode','append');
fclose(fid);

figure(2)
plot (coor_wp_lla_deg_m(1:end-1,1), coor_wp_lla_deg_m(1:end-1,2))