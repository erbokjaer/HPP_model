function bess = batteryParameters()
%BATTERYPARAMETERS  Battery plant parameters (placeholder values).
%   Sign convention: positive power = discharging (export to grid).
%   N asset instances share the plant equally. Changing N requires
%   rebuilding the model with buildHPP.

    bess.N      = 2;            % [-]   asset instances in the model
    bess.Prated = 30e6;         % [W]   converter rating, plant total (pu base)
    bess.Pmax   = 30e6;         % [W]   max charge/discharge power, plant total
    bess.E      = 60e6*3600;    % [J]   energy capacity, plant total (60 MWh)
    bess.SOC0   = 0.5;          % [-]   initial state of charge
    bess.SOCmin = 0.1;          % [-]
    bess.SOCmax = 0.9;          % [-]
end
