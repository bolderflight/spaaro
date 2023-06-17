function limited_vector = tilt_limiter (thrust_vector_raw)
    % Modify the acceleration command so that the tilt is within limit
    % It should also maintain the vertical acceleration command
    % Input:    
    %           thrust_vector_raw - raw acceleration command, can be anything
    %           angle_lim_deg - lean angle limit
    % Output:   
    %           limited_vector - limited acceleration vector
    temp = dot(thrust_vector_raw/norm(thrust_vector_raw),[0 0 -1]);
    if temp >= 0.906308 % This constant is precomputed from cosd(25)
        limited_vector = thrust_vector_raw;
    else
        a_z_new = thrust_vector_raw(3); 
        r = abs(a_z_new * 0.466308); % This constant is precomputed from tand(25)
        sqrt_temp = sqrt(thrust_vector_raw(1)^2 + thrust_vector_raw(2)^2);
        cos_phi = thrust_vector_raw(2) / sqrt_temp;
        sin_phi = thrust_vector_raw(1) / sqrt_temp;
        a_x_new = r * sin_phi;
        a_y_new = r * cos_phi;
        limited_vector = [a_x_new, a_y_new, a_z_new];
    end
end