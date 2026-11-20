function cases = bess_soc()
%BESS_SOC  State-of-charge limits of the BESS.
%   Nominal profile: the BESS discharges until the wind step at 20 s and
%   charges the renewable surplus afterwards. The HPPC derates the BESS
%   discharge (charge) linearly to zero over hppc.SOCband (0.1) above SOCmin
%   (below SOCmax); at SOCmin / SOCmax the BESS controller (REEC_C) also
%   blocks discharge / charge. The "+20 MW" cases step P_ref at 40 s, so the
%   BESS has to discharge (ramp 36 MW/min). All cases run 120 s.

    up = {"grid.Pstep.dP", 20e6, "sim.Tstop", 120};
    cases = caseList({
        "SOC0 0.5",           {}
        "SOC0 0.15",          {"bess.SOC0", 0.15}
        "at SOCmin",          {"bess.SOC0", 0.1}
        "SOC0 0.85",          {"bess.SOC0", 0.85}
        "at SOCmax",          {"bess.SOC0", 0.9}
        "SOC0 0.5, +20 MW",   up
        "SOC0 0.15, +20 MW",  [up, {"bess.SOC0", 0.15}]
    });
    cases.("sim.Tstop")(:) = 120;  % same stop time for all cases, so they plot on one time axis
end
