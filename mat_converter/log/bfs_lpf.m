classdef bfs_lpf
    properties
        freq_cutoff
        freq_samp
        delay0
        a
        b
        output
    end
    methods
        function obj = init_filter(obj, freq_samp, freq_cutoff)    
            obj.freq_cutoff = freq_cutoff;
            obj.freq_samp = freq_samp;
            fc = obj.freq_cutoff / obj.freq_samp;
            cos2pi = cos(2 * pi * fc);
            obj.b = 2 - cos2pi - sqrt(2^(2 - cos2pi) - 1);
            obj.a = 1 - obj.b;
        end

        function obj = reset_val (obj, val)
            obj.delay0 = val;
            obj.output = val;
        end

        function obj = apply_filter (obj, val)
            obj.output = obj.a * val + obj.b * obj.delay0;
            obj.delay0 = obj.output;
        end
    end
end