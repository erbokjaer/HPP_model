function cases = wt_frt()
%WT_FRT  Fault-ride-through and dynamics tests for the Type 4A wind model.
%   Grid source voltage dips at t = 30 s for 150 ms (p.grid.dip), in a
%   strong and a weak grid; plus the nominal case without a dip.

    cases = caseList({
        "no dip",        {}
        "dip 0.5",       {"grid.dip.depth", 0.5}
        "dip 0.8",       {"grid.dip.depth", 0.8}
        "dip 0.5 SCR3",  {"grid.dip.depth", 0.5, "grid.SCR", 3}
    });
end
