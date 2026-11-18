function cases = hppc_p()
%HPPC_P  Closed-loop P control at the POC (stage 5): steps in the TSO
%   power reference at t = 40 s (p.grid.Pstep), and the open-loop HPPC for
%   comparison. Before t = 20 s the wind plant has 0.6 pu available, so
%   P_ref 150 MW cannot be met (the BESS is at its discharge limit); after
%   the wind step to 0.9 pu it can.

    cases = caseList({
        "Pref 150 MW",          {}
        "step -30 MW",          {"grid.Pstep.dP", -30e6}
        "step +20 MW",          {"grid.Pstep.dP",  20e6}
        "step +40 MW (limit)",  {"grid.Pstep.dP",  40e6}
        "open loop",            {"hppc.Pflag", 0}
        "open loop, -30 MW",    {"hppc.Pflag", 0, "grid.Pstep.dP", -30e6}
    });
end
