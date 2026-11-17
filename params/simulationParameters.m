function sim = simulationParameters()
%SIMULATIONPARAMETERS  Solver and timing settings.
%   The controller sample time (p.ctrl.Ts) must be an integer multiple of
%   sim.Ts. This mirrors PSCAD, where the controller DLL runs at a
%   multiple of the EMT time step.

    sim.Ts    = 1e-3;   % [s]  base (plant) fixed step size
    sim.Tstop = 60;     % [s]  simulation stop time
    sim.logDecimation = 10;   % [-] log every 10th step (10 ms) to save memory and time
end
