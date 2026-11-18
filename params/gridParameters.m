function grid = gridParameters()
%GRIDPARAMETERS  Point of connection and grid equivalent (typical values).

    grid.Pmax_poc = 180e6;   % [W]  export limit at point of connection
    grid.Pbase    = 180e6;   % [W]  power base (= system MVA base of the network
                             %      model and the HPPC per-unit interface)
    grid.Pref     = 150e6;   % [W]  plant power reference (from TSO/market)
    grid.f0       = 50;      % [Hz] nominal frequency

    % Step in the TSO power reference (Pref + dP from t0); dP 0 = no step.
    % Case set hppc_p.
    grid.Pstep.t0 = 40;      % [s]  time of the step
    grid.Pstep.dP = 0;       % [W]  change of Pref

    % Grid Thevenin equivalent at the POC
    grid.Vpoc = 132e3;       % [V]  nominal POC voltage
    grid.SCR  = 10;          % [-]  short-circuit ratio, Ssc / Pmax_poc
    grid.XR   = 10;          % [-]  X/R ratio of the grid impedance
    grid.E    = 1.0;         % [pu] Thevenin source voltage (angle 0)

    % Grid voltage dip (source voltage of the Thevenin equivalent); depth 0
    % = no dip. Used for fault-ride-through tests (case set wt_frt).
    grid.dip.t0    = 30;     % [s]  start
    grid.dip.dur   = 0.15;   % [s]  duration
    grid.dip.depth = 0;      % [pu] reduction of E during the dip

    % Grid frequency step (pu on f0); df 0 = no event. Seen by the plant
    % controllers (network bus) and the HPPC (TSO bus). Case set freq_step.
    grid.fev.t0 = 40;        % [s]  time of the step
    grid.fev.df = 0;         % [pu] frequency change (0.004 = +0.2 Hz)
end
