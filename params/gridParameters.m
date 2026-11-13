function grid = gridParameters()
%GRIDPARAMETERS  Point of connection / grid parameters (placeholder values).

    grid.Pmax_poc = 180e6;   % [W]  export limit at point of connection
    grid.Pref     = 150e6;   % [W]  plant power reference (from TSO/market)
    grid.f0       = 50;      % [Hz] nominal frequency
end
