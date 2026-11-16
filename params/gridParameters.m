function grid = gridParameters()
%GRIDPARAMETERS  Point of connection and grid equivalent (typical values).

    grid.Pmax_poc = 180e6;   % [W]  export limit at point of connection
    grid.Pbase    = 180e6;   % [W]  power base (= system MVA base of the network
                             %      model and the HPPC per-unit interface)
    grid.Pref     = 150e6;   % [W]  plant power reference (from TSO/market)
    grid.f0       = 50;      % [Hz] nominal frequency

    % Grid Thevenin equivalent at the POC
    grid.Vpoc = 132e3;       % [V]  nominal POC voltage
    grid.SCR  = 10;          % [-]  short-circuit ratio, Ssc / Pmax_poc
    grid.XR   = 10;          % [-]  X/R ratio of the grid impedance
    grid.E    = 1.0;         % [pu] Thevenin source voltage (angle 0)
end
