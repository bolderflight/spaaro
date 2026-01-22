function m = fmod(a, b)

    % Where the mod function returns a value in region [0, b), this
    % function returns a value in the region [-b, b), with a negative
    % value only when a is negative.

    if a == 0
        m = 0;
    else
        m = mod(a, b) + (b*(sign(a) - 1)/2);
    end

end

