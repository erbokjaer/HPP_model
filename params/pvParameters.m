function pv = pvParameters()
%PVPARAMETERS  PV power plant parameters (placeholder values).
%   N asset instances (inverter groups / feeders) share the plant rating
%   equally. Changing N requires rebuilding the model with buildHPP.

    pv.N      = 2;       % [-]  asset instances in the model
    pv.Prated = 50e6;    % [W]  plant rating (all instances together)
    pv.Pavail = 0.4;     % [pu] available power (constant placeholder)
end
