function [t, v] = resultSignal(o, sig, strict)
%RESULTSIGNAL  One logged signal of a run, in plot units.
%
%   [t, v] = resultSignal(o, sig) for a Simulink.SimulationOutput o: time
%   and values scaled to MW, Mvar, pu, Hz or - (signalUnit). "PF" is
%   derived: P_poc / |S_poc|. A signal that is not logged gives empty t, v;
%   resultSignal(o, sig, true) raises an error instead, listing the logged
%   signals.

    if nargin < 3, strict = false; end
    t = [];  v = [];
    sig = string(sig);
    if sig == "PF"
        [t, P] = resultSignal(o, "P_poc", strict);  [~, Q] = resultSignal(o, "Q_poc", strict);
        v = P ./ hypot(P, Q);
        return
    end
    logged = string(o.yout.getElementNames);
    if ~ismember(sig, logged)
        if strict
            error("resultSignal:noSignal", "Signal ""%s"" is not logged. Logged signals: %s", ...
                  sig, strjoin(logged, ", "));
        end
        return
    end
    x = o.yout.get(char(sig)).Values;
    [~, k] = signalUnit(sig);
    t = x.Time;  v = squeeze(x.Data) * k;
end
