function bess = batteryParameters()
%BATTERYPARAMETERS  Battery energy storage parameters (placeholder values).
%   Sign convention: positive power = discharging (export to grid).

    bess.Pmax   = 30e6;        % [W]   max charge/discharge power
    bess.E      = 60e6*3600;   % [J]   energy capacity (60 MWh)
    bess.SOC0   = 0.5;         % [-]   initial state of charge
    bess.SOCmin = 0.1;         % [-]
    bess.SOCmax = 0.9;         % [-]
end
