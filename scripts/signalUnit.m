function [u, k] = signalUnit(sig)
%SIGNALUNIT  Plot unit of a logged signal and the scale to get there.
%
%   [u, k] = signalUnit(sig): P_* in MW and Q_* in Mvar (k = 1e-6, logged
%   in W / var), V_* in pu, f in Hz (logged in pu of 50 Hz), others "-".

    sig = string(sig);
    if startsWith(sig, "P_"),     u = "MW";   k = 1e-6;
    elseif startsWith(sig, "Q_"), u = "Mvar"; k = 1e-6;
    elseif startsWith(sig, "V_"), u = "pu";   k = 1;
    elseif sig == "f",            u = "Hz";   k = 50;
    else,                         u = "-";    k = 1;
    end
end
