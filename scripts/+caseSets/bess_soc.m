function cases = bess_soc()
%BESS_SOC  State-of-charge limits of the WECC REEC_C battery control.
%   Nominal profile: the BESS discharges until the wind step at 20 s and
%   charges afterwards. At SOCmin it may not discharge, at SOCmax it may
%   not charge (current limit logic in BESS_ctrl).

    cases = caseList({
        "SOC0 0.5",       {}
        "at SOCmin",      {"bess.SOC0", 0.1}
        "at SOCmax",      {"bess.SOC0", 0.9}
    });
end
