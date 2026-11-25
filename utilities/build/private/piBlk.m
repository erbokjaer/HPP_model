function piBlk(sys, name, Kp, Ki, lim)
    % discrete PI with clamping anti-windup; lim "" = external limits (up, lo ports)
    if lim == ""
        nb(sys, "pid", name, "Controller", "PI", "P", Kp, "I", Ki, "LimitOutput", "on", ...
            "SatLimitsSource", "external", "AntiWindupMode", "clamping", "SampleTime", "-1");
    else
        nb(sys, "pid", name, "Controller", "PI", "P", Kp, "I", Ki, "LimitOutput", "on", ...
            "UpperSaturationLimit", lim, "LowerSaturationLimit", "-" + lim, ...
            "AntiWindupMode", "clamping", "SampleTime", "-1");
    end
end
