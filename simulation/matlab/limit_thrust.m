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
