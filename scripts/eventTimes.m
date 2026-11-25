function ev = eventTimes(p)
%EVENTTIMES  The timed events in p: where they are, when, and whether active.
%
%   ev = eventTimes(p) is a struct array with one element per event group:
%     path    field path of the group in p ("grid.fev", ...)
%     t       time of the event [s]
%     active  true if the event changes anything (size not 0)
%
%   The groups are the grid and TSO events of the case sets: power and
%   reactive power reference steps (grid.Pstep, grid.Qstep), voltage dip
%   (grid.dip), frequency step (grid.fev), phase jump (grid.pjump) and the
%   disturbance of the swing-equation grid (grid.sys.dist). Every field of
%   a group acts only from its time t on (Step blocks and t >= t0 in the
%   model), so before t a run does not depend on it. eventWindow takes the
%   window around the earliest active event; operatingPoints leaves the
%   groups whose time is after the operating point out of its cache key.
%
%   Not an event here: the wind step p.wt.tStep (20 s), which is part of
%   the settling before the events, and the frequency profile p.grid.fprof.
%   An operating point taken after either of them simply depends on them.
%   A new event in the model must be added here, otherwise its parameters
%   stay in the cache key (correct, only less sharing of operating points).

    g = p.grid;
    ev = struct("path", {}, "t", {}, "active", {});
    ev(end+1) = struct("path", "grid.Pstep",    "t", g.Pstep.t0,    "active", g.Pstep.dP ~= 0);
    ev(end+1) = struct("path", "grid.Qstep",    "t", g.Qstep.t0,    "active", g.Qstep.dQ ~= 0);
    ev(end+1) = struct("path", "grid.dip",      "t", g.dip.t0,      "active", g.dip.depth ~= 0);
    ev(end+1) = struct("path", "grid.fev",      "t", g.fev.t0,      "active", g.fev.df ~= 0);
    ev(end+1) = struct("path", "grid.pjump",    "t", g.pjump.t0,    "active", g.pjump.dphi ~= 0);
    ev(end+1) = struct("path", "grid.sys.dist", "t", g.sys.dist.t0, "active", g.sys.dist.dP ~= 0);
end
