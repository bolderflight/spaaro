classdef LPF2p
    properties
        freq_cutoff
        freq_samp
        delay0
        delay1
        delay2
        a1
        a2
        b0
        b1
        b2
        output
    end
    methods
        function obj = init_filter(obj, freq_samp, freq_cutoff)    
            obj.freq_cutoff = freq_cutoff;
            obj.freq_samp = freq_samp;
            cutoff = max (obj.freq_samp * 0.001, obj.freq_cutoff);
            fr = obj.freq_samp / cutoff;
            ohm = tan (pi/ fr);
            c = 1 + 2 * cos (pi/4) * ohm + ohm * ohm;
            obj.b0 = ohm * ohm/ c;
            obj.b1 = 2 * obj.b0;
            obj.b2 = obj.b0;

            obj.a1 = 2 * (ohm * ohm - 1)/c;
            obj.a2 = (1 - 2 * cos (pi/4) * ohm + ohm * ohm) / c;
        end

        function obj = reset_val (obj, val)
            obj.delay1 = val;
            obj.delay2 = val;
            obj = obj.apply_filter (val);
        end

        function obj = apply_filter (obj, val)
            obj.delay0 = val - obj.delay1 * obj.a1 - obj.delay2 * obj.a2;
            obj.output = obj.delay0 * obj.b0 + obj.delay1 * obj.b1 + obj.delay2 * obj.b2;
            obj.delay2 = obj.delay1;
            obj.delay1 = obj.delay0;
        end
    end
end