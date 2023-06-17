function [x,y] = circular_limit(x_in, y_in)
    % Normalize square [-1,1] to unit circular limit
    r = sqrt(x_in.^2 + y_in.^2);
    if r <= 1
        x = x_in;
        y = y_in;
    else
        x = x_in ./ r;
        y = y_in ./ r;
    end
end