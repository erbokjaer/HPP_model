function [code, Tsolver] = simscapeMode(p)
%SIMSCAPEMODE  Variant code and solver step of the Simscape network mode.
%
%   [code, Tsolver] = simscapeMode(p)
%     code     1 = phasor (frequency-and-time mode), 2 = EMT (time mode);
%              the variant control variable HPP_simscape in HPP.slx (the
%              Solver Configuration and the phasor filter are variants)
%     Tsolver  [s] step of the Simscape local solver and fundamental step
%              of HPP.slx: p.sim.Ts, or p.net.emtTs with the Simscape
%              network in EMT mode (the controllers keep their rates)
%   p.net.simscapeMode ("phasor" | "emt", networkParameters) is fixed at
%   compile time (compileSignature). With the nodal network the mode has no
%   effect (code 1, step p.sim.Ts).
    mode = string(p.net.simscapeMode);
    if ~any(mode == ["phasor", "emt"])
        error("simscapeMode:mode", "p.net.simscapeMode must be ""phasor"" or ""emt"", not ""%s"".", mode);
    end
    emt = mode == "emt" && p.net.solverType == "simscape";
    code = 1 + emt;
    if emt
        Tsolver = p.net.emtTs;
        r = p.sim.Ts / Tsolver;
        if abs(r - round(r)) > 1e-9
            error("simscapeMode:step", "p.sim.Ts (%g s) must be an integer multiple of p.net.emtTs (%g s).", ...
                p.sim.Ts, Tsolver);
        end
    else
        Tsolver = p.sim.Ts;
    end
end
