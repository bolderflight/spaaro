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