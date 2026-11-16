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
end
