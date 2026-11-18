function cases = freq_step()
%FREQ_STEP  Grid frequency steps at t = 50 s (p.grid.fev), Energinet DK1;
%   70 s runs. The step comes after the plant has ramped up following the
%   wind step at 20 s (ramp limit 36 MW/min, with the FSM reserve until ~40 s).
%   Central control (default): the HPPC gives the frequency response of the
%   whole plant, on Pn = 180 MW:
%   - LFSM-O above 50.2 Hz and LFSM-U below 49.8 Hz, droop 5 %, always on;
%   - FSM (FCR delivery, hppc.FSMflag = 1): droop 5 %, deadband 10 mHz,
%     range +-10 % of Pn, with the BESS keeping that range as reserve.
%   Wind and PV run at available power, so upward response comes from the BESS.
%   "decentral": the stage-4 setup for comparison: open-loop HPPC, each
%   plant controller with its own REPC_A droop (5 %, +-20 mHz) and P loop.

    decentral = {"hppc.Pflag", 0, "hppc.Frqflag", 0, ...
                 "pc.wppc.Frqflag", 1, "pc.sppc.Frqflag", 1, "pc.bessc.Frqflag", 1};
    fsm = {"hppc.FSMflag", 1};
    T = {"grid.fev.t0", 50, "sim.Tstop", 70};
    cases = caseList({
        "no event",                T
        "+0.2 Hz (LFSM-O limit)",  [T, {"grid.fev.df",  0.004}]
        "+0.5 Hz LFSM-O",          [T, {"grid.fev.df",  0.010}]
        "-0.5 Hz LFSM-U",          [T, {"grid.fev.df", -0.010}]
        "+0.2 Hz FSM",             [T, {"grid.fev.df",  0.004}, fsm]
        "-0.2 Hz FSM",             [T, {"grid.fev.df", -0.004}, fsm]
        "+0.5 Hz FSM + LFSM-O",    [T, {"grid.fev.df",  0.010}, fsm]
        "+0.2 Hz, decentral",      [T, {"grid.fev.df",  0.004}, decentral]
        "-0.2 Hz, decentral",      [T, {"grid.fev.df", -0.004}, decentral]
    });
end
