function net = networkParameters()
%NETWORKPARAMETERS  Collector system and transformers (typical values).
%
%   Per asset instance k (one aggregated feeder):
%     asset terminal (LV) -- unit transformer -- feeder cable -- 33 kV busbar
%   then  33 kV busbar -- main transformer -- POC (132 kV) -- grid
%   The grid equivalent is in gridParameters. The solver data (Z matrix
%   etc.) are derived in networkSolverData. See docs/design.md, section 5.

    net.Vlv = 690;        % [V]  asset terminal voltage
    net.Vmv = 33e3;       % [V]  collector voltage
    % Vhv = p.grid.Vpoc

    % Unit transformers (one equivalent per asset instance; rating = instance rating)
    net.unit.uk = 0.08;   % [pu] short-circuit voltage on own rating
    net.unit.XR = 8;      % [-]

    % Feeder cables, 33 kV XLPE (per km)
    net.cable.R = 0.10;      % [ohm/km]
    net.cable.X = 0.12;      % [ohm/km]
    net.cable.C = 0.25e-6;   % [F/km]
    net.cable.len.wt   = 5.0;   % [km] per feeder
    net.cable.len.pv   = 2.0;
    net.cable.len.bess = 0.5;
    % parallel cables per asset instance (aggregated feeder): a 33 kV feeder
    % typically carries 30-40 MW, so a 75 MW wind instance needs 2
    net.cable.nPar.wt   = 2;
    net.cable.nPar.pv   = 1;
    net.cable.nPar.bess = 1;

    % Main transformer 33/132 kV
    net.main.S  = 250e6;  % [VA]
    net.main.uk = 0.12;   % [pu] on own rating
    net.main.XR = 40;     % [-]
end
