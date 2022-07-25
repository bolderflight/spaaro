function norm_out = remap_deadband(raw_in, in_min, in_max, out_min, out_max, deadband)
in_avg = (in_min + in_max) / 2;
out_avg = (out_min + out_max) / 2;

in_deadband_range = (in_max - in_min) * deadband / 2;
in_deadband_low = in_avg - in_deadband_range;
in_deadband_hi = in_avg + in_deadband_range;

if raw_in < in_deadband_low
    norm_out = (raw_in - in_deadband_low) .* (out_max - out_avg)./(in_deadband_low - in_min) + out_avg;
elseif raw_in > in_deadband_low && raw_in < in_deadband_hi
    norm_out = out_avg;
elseif raw_in > in_deadband_hi
    norm_out = (raw_in - in_deadband_hi) .* (out_max - out_avg)./(in_max - in_deadband_hi) + out_avg;
end
