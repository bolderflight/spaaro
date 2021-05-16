function ret  = constrain360(ang)
%Constrain360 Converts a +/-180 value to a 0 - 360 value
ret = fmod(ang, 360);
if ret < 0
    ret = ret + 360;
end

end

