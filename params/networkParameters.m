function net = networkParameters()
%NETWORKPARAMETERS  Collector systems and transformers (typical values).
%
%   Per plant (wt, pv, bess), every unit (asset instance) has its own
%   unit transformer and is tapped off a radial feeder:
%     unit terminal (LV) -- unit transformer -- tap on the feeder
%     plant collector bus -- segment 1 -- tap 1 -- segment 2 -- tap 2 ...
%   The collector bus is the 33 kV HPP busbar itself or, for a plant with
%   a plant transformer (plant.S > 0, wind 10/33 kV), its own collector
%   busbar behind that transformer. Then
%     33 kV HPP busbar -- HPP transformer 33/132 kV -- POC -- grid.
%   Aggregation: a unit stands for p.<type>.nAgg real units (its transformer
%   rating is scaled with nAgg); a modelled feeder stands for cable.nPar
%   identical parallel feeders (cable impedance / nPar, charging * nPar).
%   The grid equivalent is in gridParameters; the solver data are built by
%   networkSolverData. See docs/design.md, section 5.

    net.Vhpp = 33e3;          % [V]  HPP collector busbar (POC voltage: p.grid.Vpoc)

    % Network model used when a run does not choose one (default). Both are
    % in HPP.slx as variants of the Network block; the choice is made per
    % run (runCase/runSweep option Network, main.m setting network), no
    % rebuild needed:
    %   "simscape": Simscape Electrical blocks in frequency-and-time mode
    %               (buildNetworkSimscape), unit transformers as series R-L
    %   "nodal":    the nodal solver Z = inv(Y) (networkSolverData, buildNetwork)
    net.solverType = "nodal";

    % Mode of the Simscape network (compile-time, in compileSignature; each
    % mode has its own Rapid Accelerator target). Same network, only the
    % Solver Configuration differs (docs/design.md, section 14.9):
    %   "phasor": frequency-and-time mode, local solver step p.sim.Ts
    %   "emt":    time mode (three-phase instantaneous), local solver step
    %             net.emtTs; the measured voltage phasors pass a one-cycle
    %             moving average, so EMT ripple stays out of the controllers
    net.simscapeMode = "phasor";
    net.emtTs = 50e-6;        % [s]  local solver step in EMT mode (p.sim.Ts must be a multiple)

    % ---- wind: 0.69/10 kV turbines on 10 kV feeders, 10/33 kV wind transformer ----
    net.wt.Vcol  = 10e3;                                            % [V] collector voltage
    net.wt.unit  = struct("S", 5.6e6, "uk", 0.07, "XR", 7);         % per turbine, 0.69/10 kV
    net.wt.cable = struct("R", 0.05, "X", 0.095, "C", 0.50e-6, ...  % [ohm/km, F/km] 10 kV, 630 mm2 Al
        "len1", 2.0, "len", 1.0, "nPar", 15);                       % [km] bus to first tap, between taps
    net.wt.plant = struct("S", 170e6, "uk", 0.10, "XR", 30, ...     % 10/33 kV wind transformer
        "Rm", 1000, "Xm", 500);                                     % magnetizing branch [pu on S]

    % ---- PV: 0.6/33 kV inverter stations on 33 kV feeders to the HPP busbar ----
    net.pv.Vcol  = 33e3;
    net.pv.unit  = struct("S", 5.5e6, "uk", 0.06, "XR", 7);         % per station
    net.pv.cable = struct("R", 0.10, "X", 0.12, "C", 0.25e-6, ...   % 33 kV, 240 mm2 Al
        "len1", 2.0, "len", 0.5, "nPar", 1);
    net.pv.plant = struct("S", 0, "uk", 0, "XR", 1, "Rm", 1, "Xm", 1);  % none: feeders end on the HPP busbar

    % ---- BESS: one 0.69/33 kV unit with a short cable to the HPP busbar ----
    net.bess.Vcol  = 33e3;
    net.bess.unit  = struct("S", 33e6, "uk", 0.08, "XR", 8);
    net.bess.cable = struct("R", 0.05, "X", 0.10, "C", 0.35e-6, ... % 33 kV, 630 mm2 Al
        "len1", 0.3, "len", 0.3, "nPar", 1);
    net.bess.plant = struct("S", 0, "uk", 0, "XR", 1, "Rm", 1, "Xm", 1);

    % ---- HPP transformer 33/132 kV (tap changer: later) ----
    % Magnetizing branch (both network models): Rm = 1000 pu gives 0.1 %
    % no-load loss, Xm = 500 pu 0.2 % magnetizing current (typical for
    % large power transformers). The Simscape model needs a finite branch.
    net.main = struct("S", 250e6, "uk", 0.12, "XR", 40, "Rm", 1000, "Xm", 500);
end
