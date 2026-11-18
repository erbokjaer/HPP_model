function cases = wt_frt()
%WT_FRT  Fault ride-through: grid source voltage dips at t = 45 s for 150 ms
%   (p.grid.dip), in a strong and a weak grid, plus the case without a dip.
%   The dip comes after the plant has ramped up following the wind step at
%   20 s (ramp limit 36 MW/min), so it hits a steady operating point.

    t = {"grid.dip.t0", 45};
    cases = caseList({
        "no dip",        t
        "dip 0.5",       [t, {"grid.dip.depth", 0.5}]
        "dip 0.8",       [t, {"grid.dip.depth", 0.8}]
        "dip 0.5 SCR3",  [t, {"grid.dip.depth", 0.5, "grid.SCR", 3}]
    });
end
