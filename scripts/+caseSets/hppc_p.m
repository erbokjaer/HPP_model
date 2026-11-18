function cases = hppc_p()
%HPPC_P  Closed-loop P control at the POC with the ramp rate constraint
%   (Energinet DK1: 1-20 % of Pn/min, default 20 %/min = 36 MW/min on
%   180 MW): steps in the TSO power reference at t = 40 s (p.grid.Pstep),
%   120 s runs so the ramps can finish. Before t = 20 s the wind plant has
%   0.6 pu available, so P_ref 150 MW cannot be met; after the wind step to
%   0.9 pu the output ramps up to it.

    T = {"sim.Tstop", 120};
    cases = caseList({
        "Pref 150 MW",             T
        "step -30 MW",             [T, {"grid.Pstep.dP", -30e6}]
        "step +20 MW",             [T, {"grid.Pstep.dP",  20e6}]
        "step +40 MW (limit)",     [T, {"grid.Pstep.dP",  40e6}]
        "step -30 MW, 10 %/min",   [T, {"grid.Pstep.dP", -30e6, "hppc.rampUp", 0.10, "hppc.rampDn", 0.10}]
        "step -30 MW, no ramp",    [T, {"grid.Pstep.dP", -30e6, "hppc.rampFlag", 0}]
        "open loop",               [T, {"hppc.Pflag", 0}]
        "open loop, -30 MW",       [T, {"hppc.Pflag", 0, "grid.Pstep.dP", -30e6}]
    });
end
