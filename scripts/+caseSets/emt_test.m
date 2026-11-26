function cases = emt_test()
%EMT_TEST  Two short cases for quick EMT tests (Simscape network, EMT mode):
%   8 s runs with the event at 5 s, when the plant is near 140 MW (it ramps
%   up from 0 in the first seconds; the wind step at 20 s is not reached).
%   About 1 minute per case in Rapid Accelerator with the 50 us EMT step,
%   instead of ~9 minutes for a 70 s case.
%   - a 150 ms voltage dip of 0.5 pu (fault ride-through, as wt_frt);
%   - a +0.5 Hz frequency step (LFSM-O, as freq_step).
%   Use with network = "simscape", simscapeMode = "emt" and, for the
%   converter dynamics, converter = "averaged" (main.m).

    T = {"sim.Tstop", 8};
    cases = caseList({
        "dip 0.5",         [T, {"grid.dip.t0", 5, "grid.dip.depth", 0.5}]
        "+0.5 Hz LFSM-O",  [T, {"grid.fev.t0", 5, "grid.fev.df", 0.010}]
    });
end
