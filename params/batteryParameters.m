function bess = batteryParameters()
%BATTERYPARAMETERS  Battery plant parameters (placeholder values).
%   Sign convention: positive power = discharging (export to grid).

    % Electrical layout: nFeeders feeders with nPerFeeder units each; every
    % unit is one asset instance (own converter model and controller) at its
    % own network node, so N = nFeeders * nPerFeeder (derived). One unit can
    % stand for nAgg identical real units at the same place (its transformer
    % is scaled with nAgg). Changing N requires rebuilding the model with
    % buildHPP; the collector data are in networkParameters.
    % Test layout: one BESS unit at the 33 kV HPP busbar.
    bess.Pn         = 30e6;     % [W]   converter rating of one unit
    bess.nAgg       = 1;        % [-]
    bess.nFeeders   = 1;        % [-]
    bess.nPerFeeder = 1;        % [-]
    % bess.N and bess.Prated (pu base) are derived in deriveParameters
    bess.Pmax   = 30e6;         % [W]   max charge/discharge power, plant total
    bess.E      = 60e6*3600;    % [J]   energy capacity, plant total (60 MWh)
    bess.SOC0   = 0.5;          % [-]   initial state of charge
    bess.SOCmin = 0.1;          % [-]
    bess.SOCmax = 0.9;          % [-]

    % ---- WECC second-generation generic model (stage 4) ----
    % REEC_C (electrical control with state of charge) + REGC_A (converter). Names follow the WECC model specifications
    % (REEC / REGC_A). Values are typical / placeholder values from common
    % WECC example data sets, NOT from a specific inverter: check them.
    % All in pu on the asset (instance) rating; times in s.
    % electrical control (asset controller -> p.ctrl.bess.par)
    bess.wecc.ctrl.Trv    = 0.02;   % terminal voltage filter
    bess.wecc.ctrl.Vdip   = 0.90;   % voltage dip threshold (low)
    bess.wecc.ctrl.Vup    = 1.10;   % voltage dip threshold (high)
    bess.wecc.ctrl.Vref0  = 1.0;    % reference voltage for reactive current injection
    bess.wecc.ctrl.dbd1   = -0.05;  % voltage deadband, lower (Vref0 - V)
    bess.wecc.ctrl.dbd2   = 0.05;   % voltage deadband, upper
    bess.wecc.ctrl.Kqv    = 2.0;    % reactive current injection gain [pu/pu]
    bess.wecc.ctrl.Iqh1   = 1.05;   % max injected reactive current
    bess.wecc.ctrl.Iql1   = -1.05;  % min injected reactive current
    bess.wecc.ctrl.Qflag  = 1;      % 1: closed-loop Q (PI on Q_set - Q_gen), 0: open loop Iq = Q / V
    bess.wecc.ctrl.Kqp    = 0.1;    % reactive power PI, proportional gain
    bess.wecc.ctrl.Kqi    = 10;     % reactive power PI, integral gain [1/s] (about 0.1 s: fastest level of the cascade)
    bess.wecc.ctrl.Tiq    = 0.02;   % reactive current lag
    bess.wecc.ctrl.dPmax  = 99;     % P reference ramp up [pu/s]
    bess.wecc.ctrl.dPmin  = -99;    % P reference ramp down [pu/s]
    bess.wecc.ctrl.Tpord  = 0.02;   % power order lag
    bess.wecc.ctrl.Imax   = 1.1;    % max converter current
    bess.wecc.ctrl.PQflag = 0;      % current priority: 0 = Q priority, 1 = P priority
    % REEC_C state-of-charge limits: SOCmin/SOCmax above (no discharge below
    % SOCmin, no charge above SOCmax); copied to p.ctrl.bess.par

    % generator/converter (asset model -> p.bess.ap(k).gen)
    bess.wecc.gen.Tg      = 0.02;   % converter current lag
    bess.wecc.gen.Rrpwr   = 10;     % active current recovery ramp [pu/s]
    bess.wecc.gen.Iqrmax  = 99;     % reactive current ramp up [pu/s]
    bess.wecc.gen.Iqrmin  = -99;    % reactive current ramp down [pu/s]
    bess.wecc.gen.Lvplsw  = 1;      % low voltage power logic on/off
    bess.wecc.gen.Zerox   = 0.4;    % LVPL: voltage where the Ip limit is 0
    bess.wecc.gen.Brkpt   = 0.9;    % LVPL: voltage where the Ip limit reaches Lvpl1
    bess.wecc.gen.Lvpl1   = 1.22;   % LVPL: Ip limit at Brkpt
    bess.wecc.gen.Lvpnt0  = 0.4;    % LVACM: Ip = 0 below this voltage
    bess.wecc.gen.Lvpnt1  = 0.8;    % LVACM: full Ip above this voltage
end
