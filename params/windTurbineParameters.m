function wt = windTurbineParameters()
%WINDTURBINEPARAMETERS  Wind power plant parameters (placeholder values).
%
%   The plant is modelled as N asset instances. Each instance aggregates
%   nAgg identical turbines (one collector feeder); nAgg = 1 means one
%   instance per physical turbine. Changing N requires rebuilding the
%   model with buildHPP.

    wt.N      = 2;      % [-]  asset instances in the model (feeders)
    wt.nAgg   = 5;      % [-]  turbines represented by one instance
    wt.Pn     = 15e6;   % [W]  rated power per turbine
    % wt.Prated (plant rating, N*nAgg*Pn) is derived in deriveParameters

    % Placeholder available-power profile (same for all instances)
    wt.Pavail0    = 0.6;   % [pu] initial available power
    wt.PavailStep = 0.9;   % [pu] available power after step
    wt.tStep      = 20;    % [s]  time of step

    % ---- IEC 61400-27-1 Type 4A model (stage 3) ----
    % Names follow IEC 61400-27-1 where possible. Values are typical /
    % placeholder values, NOT taken from the standard or a turbine: check
    % them against the standard's parameter tables and project data.
    % All in pu on the asset (instance) rating; times in s.

    % control (asset controller WT_ctrl -> p.ctrl.wt.par)
    wt.iec.ctrl.Tufilt   = 0.01;   % voltage measurement filter
    wt.iec.ctrl.Tpfilt   = 0.01;   % active power measurement filter
    wt.iec.ctrl.Tqfilt   = 0.01;   % reactive power measurement filter
    wt.iec.ctrl.udip     = 0.90;   % voltage dip (FRT) threshold
    wt.iec.ctrl.dpmaxp4A = 1.0;    % max power order ramp rate [pu/s] (P control 4A)
    wt.iec.ctrl.Tpordp4A = 0.01;   % power order lag (P control 4A)
    wt.iec.ctrl.uref0    = 1.0;    % reference voltage for dip current injection
    wt.iec.ctrl.Kqv      = 2.0;    % additional reactive current gain during a dip [pu/pu]
    wt.iec.ctrl.iqH1     = 1.05;   % max reactive current during a dip
    wt.iec.ctrl.imax     = 1.1;    % max total current (current limitation)
    wt.iec.ctrl.MqG      = 1;      % Q control mode: 1 closed-loop reactive power, 2 open loop
    wt.iec.ctrl.kpq      = 0.1;    % reactive power PI, proportional gain [pu i / pu q]
    wt.iec.ctrl.kiq      = 5;      % reactive power PI, integral gain [1/s]

    % generator system (asset model WT_asset -> p.wt.ap(k).gen)
    wt.iec.gen.Tg     = 0.01;      % current generation time constant
    wt.iec.gen.dipmax = 1.0;       % max active current ramp rate [pu/s]
    wt.iec.gen.diqmax = 100;       % max reactive current ramp rate [pu/s]
end
