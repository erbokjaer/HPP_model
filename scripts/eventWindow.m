function win = eventWindow(cases, pBase, window)
%EVENTWINDOW  The event window of a case set: operating point and stop time.
%
%   win = eventWindow(cases, pBase, [pre post]) for the case table cases
%   (defineCases) on top of pBase (HPP_parameters, with the run's options):
%     win.tEvent  earliest active event of all cases of the set [s]
%                 (eventTimes: grid.Pstep, Qstep, dip, fev, pjump, sys.dist)
%     win.tOp     start of the window = time of the operating point:
%                 tEvent - pre, rounded down to a multiple of win.dt, and
%                 always before tEvent
%     win.post    the window ends at min(tEvent + post, p.sim.Tstop) per
%                 case; post = Inf runs to the case's stop time
%     win.dt      the slowest sample time of the model (controllers, logging),
%                 so the operating point falls on a sample of every rate and
%                 the logged samples stay on the same times as a run from 0
%
%   Event times stay absolute (the case sets and p are unchanged): the
%   window starts from a saved operating point at tOp and the logged time
%   axis runs from tOp, as in a full run. One window per case set, so the
%   cases of a set still plot on one time axis; cases without an active
%   event (the "no event" reference) get the same window.

    arguments
        cases table
        pBase (1,1) struct
        window (1,2) double {mustBeNonnegative}
    end
    tEvent = inf;
    dt = 0;
    for k = 1:height(cases)
        p = applyCase(pBase, cases(k, :));
        ev = eventTimes(p);
        ev = ev([ev.active]);
        if ~isempty(ev), tEvent = min([tEvent, ev.t]); end
        dt = max(dt, slowestRate(p));
    end
    if ~isfinite(tEvent)
        error("eventWindow:noEvent", "No case of this set has an active event (eventTimes: grid.Pstep, " + ...
            "Qstep, dip, fev, pjump, sys.dist); run it without Window.");
    end
    tOp = floor((tEvent - window(1)) / dt + 1e-9) * dt;
    if tOp >= tEvent - 1e-9, tOp = tOp - dt; end            % pre = 0: still before the event
    if tOp <= 0
        error("eventWindow:early", "The event at %g s leaves no time for an operating point " + ...
            "(window start %g s); run it without Window.", tEvent, tOp);
    end
    tOp = round(tOp, 9);
    win = struct("tEvent", tEvent, "tOp", tOp, "pre", window(1), "post", window(2), "dt", dt);
end

function dt = slowestRate(p)
    % slowest controller sample time (p.ctrl.*.Ts) and the logging interval
    dt = p.sim.Ts * p.sim.logDecimation;
    for f = string(fieldnames(p.ctrl))'
        c = p.ctrl.(f);
        if isstruct(c) && isfield(c, "Ts") && isnumeric(c.Ts) && isscalar(c.Ts)
            dt = max(dt, c.Ts);
        end
    end
end
