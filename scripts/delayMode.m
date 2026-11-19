function [p, code] = delayMode(p, cases, override)
%DELAYMODE  Resolve how the communication delays are compiled for a run.
%
%   [p, code] = delayMode(p, cases)            from p.comm.delays
%   [p, code] = delayMode(p, cases, "fixed")   override ("" = use p)
%
%   p.comm.delays (commParameters) or the override is "fixed", "variable"
%   or "auto". "auto" becomes "variable" when the case set (table cases)
%   changes a communication delay (a comm.* column), otherwise "fixed".
%   Returns p with p.comm.delays resolved to "fixed" or "variable" and the
%   variant code for HPP.slx (variant control variable HPP_delays):
%     1  fixed:    delay lengths fixed at compile time (fastest)
%     2  variable: variable-length delays, tunable up to p.comm.maxDelay

    arguments
        p (1,1) struct
        cases table
        override (1,1) string {mustBeMember(override, ["", "auto", "fixed", "variable"])} = ""
    end
    mode = p.comm.delays;
    if override ~= "", mode = override; end
    mode = string(mode);
    if ~ismember(mode, ["auto", "fixed", "variable"])
        error("delayMode:value", "p.comm.delays must be ""auto"", ""fixed"" or ""variable"", not ""%s"".", mode);
    end
    if mode == "auto"
        cols = string(cases.Properties.VariableNames);
        sweeps = any(startsWith(cols, "comm.") & ~ismember(cols, ["comm.maxDelay", "comm.delays"]));
        if sweeps, mode = "variable"; else, mode = "fixed"; end
    end
    p.comm.delays = mode;
    code = 1 + double(mode == "variable");
end
