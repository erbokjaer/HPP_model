function hppc = hppcParameters()
%HPPCPARAMETERS  Settings of the HPP controller (HPPC), stage 5.
%
%   Copied into p.ctrl.hppc.par by controllerParameters. Per unit on the
%   POC base p.grid.Pbase (180 MW = Pn of the plant at the POC), frequency
%   in pu of 50 Hz, times in s.
%
%   Grid code: Energinet requirements for DK1, from the Danish RfG
%   requirements (Krav jf. RfG, bilag 1; power park modules, type D) and
%   Technical Regulation 3.3.1 (energy storage facilities, rev. 6). Ranges
%   are noted per parameter; defaults are the DK1 default settings.
%
%   Central control (default): the HPPC closes the P loop at the POC and
%   gives the frequency response; the plant controllers pass their P
%   set-points on (p.pc.<plant>.Frqflag = 0: no local droop, no plant P
%   loop). Their Q loops stay closed.
%   Decentral (stage-4 behaviour, for comparison): Pflag = 0, Frqflag = 0
%   here and Frqflag = 1 in the plant controllers.
%
%   Q / V / PF: the HPPC closes a Q loop (RefFlag 0), a voltage loop with
%   slope (RefFlag 1) or a power factor loop (RefFlag 2) at the POC over the
%   plants' own Q loops, and splits its Q order over the plants by rating.
%
%   Override per case, e.g. {"hppc.FSMflag", 1}.

    % ---- P control at the POC ----
    hppc.Pflag   = 1;        % 1: closed-loop P at the POC (PI on P_target - P_poc), 0: open loop
    hppc.Tp      = 0.2;      % P_poc measurement filter (>= Ts = 0.1 s)
    hppc.Tref    = 0.4;      % reference filter on the target in the PI error: the PI
                             % corrects only what the plant does not follow (losses,
                             % deviations), not the response time of a step
    hppc.Vfrz    = 0.9;      % PIs frozen while V_poc < Vfrz (voltage dip) ...
    hppc.Thld    = 1.0;      % ... and for Thld after it (assets recover their P)
    hppc.Kpg     = 0.1;      % P PI, proportional gain [pu / pu]
    hppc.Kig     = 0.5;      % P PI, integral gain [1/s]
    hppc.femax   = 0.2;      % max P error
    hppc.femin   = -0.2;     % min P error

    % ---- Ramp rate constraint (power gradient) on the P set-point ----
    % RfG 15(6)(e): 1-20 % of Pn/min up and down, at most 60 MW/min; TR 3.3.1
    % section 32: 1-20 %/min, at most 1 MW/s, maximum gradient by default.
    % Applies to the set-point and to availability changes, not to the
    % frequency response (priority: frequency control before power and
    % gradient constraints, TR 3.3.1 section 26).
    hppc.rampFlag = 1;       % 1: ramp rate constraint on, 0: off
    hppc.rampUp   = 0.20;    % [pu of Pn per minute] (20 %/min of 180 MW = 36 MW/min)
    hppc.rampDn   = 0.20;

    % ---- Frequency response (droop on Pn, Energinet DK1) ----
    hppc.Frqflag  = 1;       % 1: frequency response in the HPPC (central), 0: none
    hppc.Freq_ref = 1.0;     % nominal frequency [pu]
    % LFSM-O (always on): threshold 50.20-50.50 Hz (DK1 50.20), droop 2-12 % (DK1 5 %)
    hppc.fO      = 50.2 / 50;
    hppc.sO      = 0.05;
    % LFSM-U (always on): threshold 49.50-49.80 Hz (DK1 49.80), droop 2-12 % (DK1 5 %)
    hppc.fU      = 49.8 / 50;
    hppc.sU      = 0.05;
    % FSM (when the plant delivers FCR): droop 2-12 %, deadband 0-500 mHz,
    % range 1.5-10 % of Pn (minimum), insensitivity <= 10 mHz, full activation <= 30 s
    hppc.FSMflag  = 0;       % 1: FSM active; the BESS keeps fsmRange as upward reserve
    hppc.sFSM     = 0.05;
    hppc.fsmDb    = 0.010 / 50;   % 10 mHz
    hppc.fsmRange = 0.10;    % [pu of Pn]

    % ---- SOC-aware BESS limits ----
    hppc.SOCband = 0.1;      % discharge (charge) derated to zero over SOCband above
                             % SOCmin (below SOCmax); SOCmin / SOCmax from p.bess

    % ---- Q / V / PF control at the POC (RfG article 21(3)(d)) ----
    hppc.Qflag   = 1;        % 1: closed loop (PI), 0: open loop (Q_ref split over the plants)
    hppc.RefFlag = 0;        % 0: Q control (TSO Q_ref), 1: voltage control, 2: power factor control
    hppc.Vref    = 1.0;      % voltage set-point [pu] (0.95-1.05)
    hppc.Vslope  = 0.04;     % voltage control slope: V change for a Q change of Qmax (2-7 %)
    hppc.PF      = 1.0;      % power factor set-point (RefFlag 2)
    hppc.PFsign  = 1;        % +1: Q export with P (over-excited), -1: Q import
    hppc.Tq      = 0.2;      % Q_poc and V_poc measurement filter (>= Ts)
    hppc.TrefQ   = 0.4;      % reference filter on the Q reference in the PI error
    hppc.Kpq     = 0.1;      % Q PI, Q and PF control: proportional gain [pu Q / pu Q]
    hppc.Kiq     = 0.5;      % Q PI, Q and PF control: integral gain [1/s]
    % voltage control PI: gains divided by the loop gain (Vslope / Qmax + dVdQ),
    % i.e. normalised loop gains; dVdQ = 1 / SCR (pu V per pu Q on 180 MVA) is a
    % commissioning setting for the grid strength at the POC
    hppc.KpV     = 0.5;      % normalised proportional gain [-]
    hppc.KiV     = 2.5;      % normalised integral gain [1/s]
    hppc.SCRset  = NaN;      % grid strength assumed by the voltage control (dVdQ = 1 / SCRset);
                             % NaN: the actual p.grid.SCR (set at commissioning). With
                             % SCRset 10 on an SCR 3 grid: 50-90 % overshoot, settled
                             % within 1.5 s; SCRset 3 on SCR 10: t90 1.4-2.1 s (> t1)
    hppc.eqmax   = 0.2;      % max Q / V error (pu Q or pu V)
    hppc.eqmin   = -0.2;
    hppc.Qmax    = 0.33;     % Q order limits at the POC [pu on 180 MVA] (about 60 Mvar,
    hppc.Qmin    = -0.33;    % power factor 0.95 at 180 MW)
end
