function cases = freq_step()
%FREQ_STEP  Grid frequency steps at t = 40 s (p.grid.fev).
%   Central control (default): the HPPC gives the frequency response of the
%   whole plant (droop Ddn / Dup with deadband on the POC base); the plant
%   controllers pass their set-points on. Wind and PV run at available
%   power, so under-frequency is covered by the BESS.
%   "decentral": the stage-4 setup for comparison: open-loop HPPC, each
%   plant controller with its own REPC_A droop and P loop.

    decentral = {"hppc.Pflag", 0, "hppc.Frqflag", 0, ...
                 "pc.wppc.Frqflag", 1, "pc.sppc.Frqflag", 1, "pc.bessc.Frqflag", 1};
    cases = caseList({
        "no event",            {}
        "+0.2 Hz",             {"grid.fev.df",  0.004}
        "-0.2 Hz",             {"grid.fev.df", -0.004}
        "+0.2 Hz, no droop",   {"grid.fev.df",  0.004, "hppc.Frqflag", 0}
        "+0.2 Hz, decentral",  [{"grid.fev.df",  0.004}, decentral]
        "-0.2 Hz, decentral",  [{"grid.fev.df", -0.004}, decentral]
    });
end
