function cases = pll_scr()
%PLL_SCR  PLL bandwidth against grid strength (stage 7, RMS tier).
%   A 20 deg phase jump of the grid source at 40 s (p.grid.pjump) excites
%   the asset PLLs (p.pll.asset) and the frequency measurement; the same
%   jump at every bandwidth (2, 5, 10, 20 Hz, prefilter 20 ms) and grid
%   strength (SCR 10, 3, 1.5). Shows where the RMS PLL loop loses damping:
%   the prefilter delay (10 ms) limits the bandwidth, and a weak grid
%   couples the injected current back into the terminal angle. RMS is only
%   approximate for fast PLLs (docs/design.md, section 14.1); the dq tier
%   (stage 8) repeats this map. All cases 60 s.

    cases = caseGrid("pll.asset.fBW", [2 5 10 20], "grid.SCR", [10 3 1.5]);
    cases.("grid.pjump.dphi")(:) = 20 * pi / 180;
    cases.("sim.Tstop")(:) = 60;
end
