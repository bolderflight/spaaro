close all
clear
clc

%TODO: 
% Zero case
% positive accel case verification

max_body_accel = -1.8;

%% Generate truth data
inertial_z = [0; 0; 1];
thrust_vector_body = [0; 0; -1.7];
truth_ypr = [deg2rad(0) deg2rad(-10) deg2rad(0)];
truth_ypr_deg = truth_ypr * 180/pi;
rotMatZYX = eul2rotm(truth_ypr);
thrust_vector_inertial = rotMatZYX * thrust_vector_body;
commanded_acc = thrust_vector_inertial;
commanded_acc(3) = commanded_acc(3) + 1;
quaternion_truth = quaternion(truth_ypr,"euler","ZYX","frame");

%% Apply tilt limit
thrust_vector_inertial_thrust_limited = limit_thrust(thrust_vector_inertial, 1.8);
limited_thrust = norm(thrust_vector_inertial_thrust_limited);

thrust_vector_inertial_tilt_limited = limit_tilt(thrust_vector_inertial_thrust_limited,25);
tilt_limited_thrust = norm(thrust_vector_inertial_tilt_limited);
tilt_angle = acos(dot(thrust_vector_inertial_tilt_limited/norm(thrust_vector_inertial_tilt_limited),[0 0 -1]));
tilt_angle = rad2deg(tilt_angle);

test_psi = deg2rad(180);
[test_phi, test_theta] = calc_tilt(thrust_vector_inertial_tilt_limited, test_psi);

test_ypr = [test_psi, test_theta,test_phi];
quaternion_test = quaternion(test_ypr,"euler","ZYX","frame");
test_rotMatZYX = eul2rotm(test_ypr);
thrust_vector_inertial_test = test_rotMatZYX * thrust_vector_body/norm(thrust_vector_body);

f = figure();
plot3 ([0,thrust_vector_inertial(1)],[0,thrust_vector_inertial(2)],...
    [0,thrust_vector_inertial(3)],'blue--o','LineWidth',2)
hold on
plot3 ([0,thrust_vector_inertial_thrust_limited(1)],[0,thrust_vector_inertial_thrust_limited(2)],...
    [0,thrust_vector_inertial_thrust_limited(3)],'g*','LineWidth',3)
plot3 ([0,thrust_vector_inertial_tilt_limited(1)],[0,thrust_vector_inertial_tilt_limited(2)],...
    [0,thrust_vector_inertial_tilt_limited(3)],'c','LineWidth',2)
%plot3 ([0,thrust_vector_inertial_test(1)],[0,thrust_vector_inertial_test(2)],...
%    [0,thrust_vector_inertial_test(3)],'r')
plot3 ([0,thrust_vector_inertial(1)],[0,thrust_vector_inertial(2)],...
    [0,thrust_vector_inertial(3) + 1],'blue','LineWidth',1)
plot3 ([0,0],[0,0],[-100, 100],'k--')
plot3 ([-100, 100],[0,0],[0,0],'k--')
plot3 ([0, 0],[-100,100],[0,0],'k--')
xlabel("x")
ylabel("y")
zlabel("z")
f.CurrentAxes.ZDir = 'Reverse';
f.CurrentAxes.YDir = 'Reverse';
grid on
grid minor
xlim([-4 4])
ylim([-4 4])
zlim([-4 4])

figure()
poseplot(quaternion_truth, [0 0 0])
hold on 
poseplot(quaternion_test, [0 0 0])
xlabel("X")
ylabel("Y")
zlabel("Z")

function limited_vector = limit_thrust(thrust_vector_raw, max_thrust)
    % Limit the acceleration command to the max thrust achievable by the
    % aircraft. Should prioritize vertical thrust
    % Input:
    %           thrust_vector_raw - raw acceleration command
    %           max_thrust - max thrust by aircraft. In term of G
    limited_vector = thrust_vector_raw;
    mag = norm(thrust_vector_raw);
    % Ignore if the acc_cmd is already achieble
    if mag < max_thrust
        limited_vector = thrust_vector_raw;
    else
        if thrust_vector_raw(3) < -max_thrust
            limited_vector(3) = -max_thrust;
        end
        hor_acc = sqrt(max_thrust^2 - limited_vector(3)^2);
        limited_vector(1) = thrust_vector_raw(1) * hor_acc/mag;
        limited_vector(2) = thrust_vector_raw(2) * hor_acc/mag;
    end

end

function limited_vector = limit_tilt (thrust_vector_raw, angle_lim_deg)
    % Modify the acceleration command so that the tilt is within limit
    % It should also maintain the vertical acceleration command
    % Input:    
    %           thrust_vector_raw - raw acceleration command, can be anything
    %           angle_lim_deg - lean angle limit
    % Output:   
    %           limited_vector - limited acceleration vector
    temp = dot(thrust_vector_raw/norm(thrust_vector_raw),[0 0 -1]);
    if temp >= cosd(angle_lim_deg) % Can precompute cos(angle_lim) to reduce compute time
        limited_vector = thrust_vector_raw;
    else
        a_z_new = thrust_vector_raw(3); 
        r = abs(a_z_new * tand(angle_lim_deg)); % Can also precompute tand(angle_lim_deg)
        sqrt_temp = sqrt(thrust_vector_raw(1)^2 + thrust_vector_raw(2)^2);
        cos_phi = thrust_vector_raw(2) / sqrt_temp;
        sin_phi = thrust_vector_raw(1) / sqrt_temp;
        a_x_new = r * sin_phi;
        a_y_new = r * cos_phi;
        limited_vector = [a_x_new, a_y_new, a_z_new];
    end
end

%% Math is weird here
function [phi_rad, theta_rad] = calc_tilt (thrust_vector_ned, psi_rad)
    % Calculate the pitch and roll of the aircraft required to tilt the
    % thrust vector to match independent of its heading. 
    % Input:    
    %           thrust_vector_ned - commanded acceleration vector in NED
    %           frame. Should be unit vector
    %           psi_rad - the current heading angle of the aircraft. 
    % Output:   
    %           phi_rad, theta_rad - calculated pitch and roll angle
    %           requires for the thrust vector to match with the commanded
    %           inertial vector
    thrust_vector_ned = thrust_vector_ned/norm(thrust_vector_ned);
    cpsi = cos(psi_rad);
    spsi = sin(psi_rad);
    theta_num = cpsi * thrust_vector_ned(1) + spsi * thrust_vector_ned(2);
    theta_denum = thrust_vector_ned(3);
    theta_rad = atan2(theta_num, theta_denum) + pi;
    ctheta = cos(theta_rad);
    stheta = sin(theta_rad);
    phi_num = stheta * cpsi * thrust_vector_ned(1) + ...
        stheta * spsi * thrust_vector_ned(2) + ctheta * thrust_vector_ned(3);
    phi_denum = spsi * thrust_vector_ned(1) - cpsi * thrust_vector_ned(2);
    phi_rad = atan2(phi_denum, phi_num) + pi;
end

