function sim = simulationParameters()
%SIMULATIONPARAMETERS  Solver and timing settings.
%   The controller sample time (p.ctrl.Ts) must be an integer multiple of
%   sim.Ts. This mirrors PSCAD, where the controller DLL runs at a
%   multiple of the EMT time step.

    sim.Ts    = 1e-3;   % [s]  base (plant) fixed step size
    sim.Tstop = 60;     % [s]  simulation stop time
end
