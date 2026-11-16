function cases = freq_step()
%FREQ_STEP  Grid frequency steps at t = 40 s (p.grid.fev) for the REPC_A
%   frequency response of the plant controllers (droop Ddn / Dup with
%   deadband fdbd). Wind and PV run at available power, so they respond to
%   over-frequency only; the BESS responds in both directions.

    cases = caseList({
        "no event",          {}
        "+0.2 Hz",           {"grid.fev.df",  0.004}
        "-0.2 Hz",           {"grid.fev.df", -0.004}
        "+0.2 Hz, no droop", {"grid.fev.df",  0.004, "pc.wppc.Frqflag", 0, ...
                              "pc.sppc.Frqflag", 0, "pc.bessc.Frqflag", 0}
    });
end
