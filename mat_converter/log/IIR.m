classdef IIR
    properties
        freq_cutoff
        freq_samp
        delay0
        a
        b
        output
    end
    methods
        function obj = init_filter(obj, freq_samp, freq_cutoff, init_val)    
            obj.freq_cutoff = freq_cutoff;
            obj.freq_samp = freq_samp;
            cutoff = freq_cutoff/freq_samp;
            obj.b = 2 - cos(2*pi*cutoff) - sqrt((2-cos(2*pi*cutoff)^2)-1);
            obj.a = 1-obj.b;
            obj.delay0 = init_val;
        end

        function obj = apply_filter (obj, val)
            obj.output = obj.a * val + obj.b * obj.delay0;
            obj.delay0 = obj.output;
        end
    end
end