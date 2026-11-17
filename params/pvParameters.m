function pv = pvParameters()
%PVPARAMETERS  PV power plant parameters (placeholder values).

    % Electrical layout: nFeeders feeders with nPerFeeder units each; every
    % unit is one asset instance (own converter model and controller) at its
    % own network node, so N = nFeeders * nPerFeeder (derived). One unit can
    % stand for nAgg identical real units at the same place (its transformer
    % is scaled with nAgg). Changing N requires rebuilding the model with
    % buildHPP; the collector data are in networkParameters.
    % Test layout: 2 feeders with 1 unit of 5 inverter stations each = 50 MW.
    pv.Pn         = 5e6;    % [W]  rated power per inverter station
    pv.nAgg       = 5;      % [-]  stations represented by one unit
    pv.nFeeders   = 2;      % [-]  feeders in the model
    pv.nPerFeeder = 1;      % [-]  units along each feeder
    % pv.N and pv.Prated are derived in deriveParameters
    pv.Pavail = 0.4;     % [pu] available power (constant placeholder)

    % ---- WECC second-generation generic model (stage 4) ----
    % REEC_B (electrical control) + REGC_A (converter). Names follow the WECC model specifications
    % (REEC / REGC_A). Values are typical / placeholder values from common
    % WECC example data sets, NOT from a specific inverter: check them.
    % All in pu on the asset (instance) rating; times in s.
    % electrical control (asset controller -> p.ctrl.pv.par)
    pv.wecc.ctrl.Trv    = 0.02;   % terminal voltage filter
    pv.wecc.ctrl.Vdip   = 0.90;   % voltage dip threshold (low)
    pv.wecc.ctrl.Vup    = 1.10;   % voltage dip threshold (high)
    pv.wecc.ctrl.Vref0  = 1.0;    % reference voltage for reactive current injection
    pv.wecc.ctrl.dbd1   = -0.05;  % voltage deadband, lower (Vref0 - V)
    pv.wecc.ctrl.dbd2   = 0.05;   % voltage deadband, upper
    pv.wecc.ctrl.Kqv    = 2.0;    % reactive current injection gain [pu/pu]
    pv.wecc.ctrl.Iqh1   = 1.05;   % max injected reactive current
    pv.wecc.ctrl.Iql1   = -1.05;  % min injected reactive current
    pv.wecc.ctrl.Qflag  = 1;      % 1: closed-loop Q (PI on Q_set - Q_gen), 0: open loop Iq = Q / V
    pv.wecc.ctrl.Kqp    = 0.1;    % reactive power PI, proportional gain
    pv.wecc.ctrl.Kqi    = 5;      % reactive power PI, integral gain [1/s]
    pv.wecc.ctrl.Tiq    = 0.02;   % reactive current lag
    pv.wecc.ctrl.dPmax  = 99;     % P reference ramp up [pu/s]
    pv.wecc.ctrl.dPmin  = -99;    % P reference ramp down [pu/s]
    pv.wecc.ctrl.Tpord  = 0.02;   % power order lag
    pv.wecc.ctrl.Imax   = 1.1;    % max converter current
    pv.wecc.ctrl.PQflag = 0;      % current priority: 0 = Q priority, 1 = P priority

    % generator/converter (asset model -> p.pv.ap(k).gen)
    pv.wecc.gen.Tg      = 0.02;   % converter current lag
    pv.wecc.gen.Rrpwr   = 10;     % active current recovery ramp [pu/s]
    pv.wecc.gen.Iqrmax  = 99;     % reactive current ramp up [pu/s]
    pv.wecc.gen.Iqrmin  = -99;    % reactive current ramp down [pu/s]
    pv.wecc.gen.Lvplsw  = 1;      % low voltage power logic on/off
    pv.wecc.gen.Zerox   = 0.4;    % LVPL: voltage where the Ip limit is 0
    pv.wecc.gen.Brkpt   = 0.9;    % LVPL: voltage where the Ip limit reaches Lvpl1
    pv.wecc.gen.Lvpl1   = 1.22;   % LVPL: Ip limit at Brkpt
    pv.wecc.gen.Lvpnt0  = 0.4;    % LVACM: Ip = 0 below this voltage
    pv.wecc.gen.Lvpnt1  = 0.8;    % LVACM: full Ip above this voltage
end
