function hppc = hppcParameters()
%HPPCPARAMETERS  Settings of the HPP controller (HPPC), stage 5.
%
%   Copied into p.ctrl.hppc.par by controllerParameters. Per unit on the
%   POC base p.grid.Pbase (180 MW), frequency in pu of 50 Hz, times in s.
%
%   Central control (default): the HPPC closes the P loop at the POC and
%   gives the frequency response; the plant controllers pass their P
%   set-points on (p.pc.<plant>.Frqflag = 0: no local droop, no plant P
%   loop). Their Q loops stay closed.
%   Decentral (stage-4 behaviour, for comparison): Pflag = 0, Frqflag = 0
%   here and Frqflag = 1 in the plant controllers.
%
%   Q / V: the HPPC closes a Q loop (RefFlag 0) or a voltage loop (RefFlag 1)
%   at the POC over the plants' own Q loops, and splits its Q order over
%   the plants by rating.
%
%   Override per case, e.g. {"hppc.Frqflag", 0}.

    hppc.Pflag   = 1;        % 1: closed-loop P at the POC (PI on P_target - P_poc), 0: open loop
    hppc.Frqflag = 1;        % 1: frequency response in the HPPC (central), 0: none
    hppc.Tp      = 0.2;      % P_poc measurement filter (>= Ts = 0.1 s)
    hppc.Tref    = 0.4;      % reference filter on the target in the PI error: the PI
                             % corrects only what the plant does not follow (losses,
                             % deviations), not the response time of a step
                             % (measurement filter, delays, plant lags)
    hppc.Kpg     = 0.1;      % P PI, proportional gain [pu / pu]
    hppc.Kig     = 0.5;      % P PI, integral gain [1/s]
    hppc.Vfrz    = 0.9;      % P PI frozen while V_poc < Vfrz (voltage dip) ...
    hppc.Thld    = 1.0;      % ... and for Thld after it (assets recover their P)
    hppc.femax   = 0.2;      % max P error
    hppc.femin   = -0.2;     % min P error

    % frequency response, droop on the POC base (5 %: 20 pu P / pu f,
    % 0.072 pu = 13 MW for +0.2 Hz outside the deadband)
    hppc.Freq_ref = 1.0;     % frequency reference [pu]
    hppc.fdbd1   = -0.0004;  % deadband, lower [pu] (-20 mHz)
    hppc.fdbd2   = 0.0004;   % deadband, upper [pu]
    hppc.Ddn     = 20;       % droop for over-frequency [pu P / pu f]
    hppc.Dup     = 20;       % droop for under-frequency (headroom: BESS)

    % Q / V control at the POC (over the plants' own closed Q loops)
    hppc.Qflag   = 1;        % 1: closed loop (PI), 0: open loop (Q_ref split over the plants)
    hppc.RefFlag = 0;        % 0: Q control (track TSO Q_ref), 1: voltage control (Vref, droop Kc)
    hppc.Vref    = 1.0;      % voltage reference at the POC [pu] (RefFlag = 1)
    hppc.Kc      = 0.0;      % reactive droop in voltage control [pu V / pu Q]
    hppc.Tq      = 0.2;      % Q_poc and V_poc measurement filter (>= Ts)
    hppc.TrefQ   = 0.4;      % reference filter on Q_ref in the PI error (as Tref)
    hppc.Kpq     = 0.1;      % Q PI, Q control: proportional gain [pu Q / pu Q]
    hppc.Kiq     = 0.5;      % Q PI, Q control: integral gain [1/s]
    hppc.KpV     = 2.0;      % Q PI, voltage control: proportional gain [pu Q / pu V]
    hppc.KiV     = 10;       % Q PI, voltage control: integral gain [pu Q / pu V / s]
                             % (loop gain dV/dQ = 1/SCR: 0.1 at SCR 10, 0.33 at SCR 3;
                             % KiV 10: 2.9 s at SCR 10, 1.0 s at SCR 3; KiV 20 oscillates at SCR 3)
    hppc.eqmax   = 0.2;      % max Q / V error (pu Q or pu V)
    hppc.eqmin   = -0.2;
    hppc.Qmax    = 0.33;     % Q order limits at the POC [pu on 180 MVA] (about 60 Mvar,
    hppc.Qmin    = -0.33;    % power factor 0.95 at 180 MW)
end
