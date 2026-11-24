function nb(sys, kind, name, varargin)
    % new block of a standard size (placed later by rowChain)
    switch kind
        case "sum",     lib = "simulink/Math Operations/Sum";            sz = [30 40];
            varargin = [varargin, {"IconShape", "rectangular"}];
        case "gain",    lib = "simulink/Math Operations/Gain";           sz = [50 30];
        case "div",     lib = "simulink/Math Operations/Divide";         sz = [35 40];
        case "minmax",  lib = "simulink/Math Operations/MinMax";         sz = [35 40];
        case "sumel",   lib = "simulink/Math Operations/Sum of Elements"; sz = [35 30];
        case "lag",     lib = "simulink/Discrete/Transfer Fcn First Order"; sz = [70 34];
        case "pid",     lib = "simulink/Discrete/Discrete PID Controller"; sz = [70 80];
        case "switch",  lib = "simulink/Signal Routing/Switch";           sz = [40 80];
        case "sat",     lib = "simulink/Discontinuities/Saturation";      sz = [40 30];
        case "satdyn",  lib = "simulink/Discontinuities/Saturation Dynamic"; sz = [50 80];
        case "rate",    lib = "simulink/Discontinuities/Rate Limiter";    sz = [40 30];
        case "dz",      lib = "simulink/Discontinuities/Dead Zone";       sz = [40 30];
        case "rel",     lib = "simulink/Logic and Bit Operations/Relational Operator"; sz = [35 40];
        case "or",      lib = "simulink/Logic and Bit Operations/Logical Operator";    sz = [35 40];
        case "term",    lib = "simulink/Sinks/Terminator";                sz = [20 20];
        case "mem",     lib = "simulink/Discrete/Memory";                 sz = [30 30];
        case "tf",      lib = "simulink/Continuous/Transfer Fcn";         sz = [70 34];
        case "int",     lib = "simulink/Continuous/Integrator";           sz = [30 30];
        case "dint",    lib = "simulink/Discrete/Discrete-Time Integrator"; sz = [30 30];
        case "udelay",  lib = "simulink/Discrete/Unit Delay";             sz = [35 30];
        case "delay",   lib = "simulink/Discrete/Delay";                  sz = [45 50];
        case "trig",    lib = "simulink/Math Operations/Trigonometric Function"; sz = [45 30];
        case "math",    lib = "simulink/Math Operations/Math Function";   sz = [45 34];
        case "bias",    lib = "simulink/Math Operations/Bias";            sz = [45 30];
        case "prod",    lib = "simulink/Math Operations/Product";         sz = [30 40];
        case "abs",     lib = "simulink/Math Operations/Abs";             sz = [35 30];
        case "ri2c",    lib = "simulink/Math Operations/Real-Imag to Complex"; sz = [40 40];
        case "c2ri",    lib = "simulink/Math Operations/Complex to Real-Imag"; sz = [40 30];
        case "ma2c",    lib = "simulink/Math Operations/Magnitude-Angle to Complex"; sz = [40 30];
        case "c2ma",    lib = "simulink/Math Operations/Complex to Magnitude-Angle"; sz = [40 30];
        case "lut",     lib = "simulink/Lookup Tables/1-D Lookup Table";  sz = [60 40];
        case "dtc",     lib = "simulink/Signal Attributes/Data Type Conversion"; sz = [45 30];
    end
    add_block(lib, sys + "/" + name, "Position", [0 0 sz], varargin{:});
end
