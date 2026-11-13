function wt = windTurbineParameters()
%WINDTURBINEPARAMETERS  Wind farm / turbine parameters (placeholder values).

    wt.N    = 10;          % [-]  number of turbines
    wt.Pn   = 15e6;        % [W]  rated power per turbine
    wt.Prated = wt.N*wt.Pn;% [W]  rated farm power

    % Placeholder available-power profile used by the skeleton plant
    wt.Pavail0    = 0.6;   % [pu] initial available power
    wt.PavailStep = 0.9;   % [pu] available power after step
    wt.tStep      = 20;    % [s]  time of step
end
