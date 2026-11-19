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

    % TSO reactive power reference (+ = export) with an optional step.
    % Case set hppc_qv.
    grid.Qref     = 0;       % [var]
    grid.Qstep.t0 = 40;      % [s]
    grid.Qstep.dQ = 0;       % [var] change of Qref

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

    % Imposed frequency profile (sys.swing = 0): piecewise linear deviation
    % from f0, pu, added to the step above. 10 points (fixed size); unused
    % points at large times. For the Energinet test profiles.
    grid.fprof.t  = (0:9) * 1e4;   % [s]  increasing
    grid.fprof.df = zeros(1, 10);  % [pu]

    % System frequency model (sys.swing = 1): the grid source is an
    % aggregated system behind the grid impedance, whose frequency follows a
    % swing equation, in pu on Ssys:
    %   2H dw/dt = Pm + dP_hpp - dP_dist - D w
    %   governor + turbine: xg' = (-w/R - xg)/Tg,  Pm' = (xg - Pm)/Tt
    % dP_hpp: change of P_poc since tOn (the system is balanced at tOn).
    % The grid angle theta = integral of 2 pi f0 w rotates the source in
    % both modes. Nordic-like per-unit values on a small system (about 2 GW)
    % so the plant's effect is visible; dist.dP 3.5 % ~ the Nordic
    % dimensioning incident (1.4 GW of 40 GW).
    grid.sys.swing = 0;      % 0: imposed frequency (fev, fprof), 1: swing equation
    grid.sys.Ssys  = 2e9;    % [VA] system rating
    grid.sys.H     = 4;      % [s]  inertia constant on Ssys
    grid.sys.D     = 1;      % [pu P / pu f] load damping
    grid.sys.R     = 0.2;    % [pu f / pu P] governor droop (FCR)
    grid.sys.Tg    = 0.5;    % [s]  governor time constant
    grid.sys.Tt    = 5;      % [s]  turbine / FCR activation time constant
    grid.sys.tOn   = 39;     % [s]  swing equation starts; the HPP ramps up until ~38 s
    grid.sys.dist.t0 = 40;   % [s]  disturbance (loss of generation)
    grid.sys.dist.dP = 0;    % [pu of Ssys] + = loss of generation / load increase
end
