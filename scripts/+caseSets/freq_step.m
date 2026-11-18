function cases = freq_step()
%FREQ_STEP  Grid frequency steps at t = 40 s (p.grid.fev), Energinet DK1.
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
    cases = caseList({
        "no event",                {}
        "+0.2 Hz (LFSM-O limit)",  {"grid.fev.df",  0.004}
        "+0.5 Hz LFSM-O",          {"grid.fev.df",  0.010}
        "-0.5 Hz LFSM-U",          {"grid.fev.df", -0.010}
        "+0.2 Hz FSM",             [{"grid.fev.df",  0.004}, fsm]
        "-0.2 Hz FSM",             [{"grid.fev.df", -0.004}, fsm]
        "+0.5 Hz FSM + LFSM-O",    [{"grid.fev.df",  0.010}, fsm]
        "+0.5 Hz, no response",    {"grid.fev.df",  0.010, "hppc.Frqflag", 0}
        "+0.2 Hz, decentral",      [{"grid.fev.df",  0.004}, decentral]
        "-0.2 Hz, decentral",      [{"grid.fev.df", -0.004}, decentral]
    });
end
