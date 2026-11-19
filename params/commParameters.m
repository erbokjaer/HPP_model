function comm = commParameters()
%COMMPARAMETERS  Communication delays between control levels (typical values).
%   Applied in HPP.slx as a discrete delay of round(T / p.sim.Ts) steps
%   (p.comm.steps, computed in deriveParameters); the receiving controller
%   samples the delayed signal at its own rate. All in seconds.
%   Set all to 0 to compare with a model without communication delays.
%
%   comm.delays chooses how the delays are compiled:
%     "fixed"    the delay lengths are fixed when the model is compiled:
%                fastest (Rapid Accelerator ~1.5 s per 60 s case), but a
%                case with other delays needs a rebuild / recompile
%     "variable" variable-length delays: the delays are tunable between runs
%                (Fast Restart, Rapid Accelerator) up to maxDelay, for delay
%                sweeps; about 0.6-1 s slower per 60 s case in Rapid Accelerator
%     "auto"     (default) "variable" when the case set changes a delay
%                (a comm.* column), otherwise "fixed"
%   runCase / runSweep / main.m can override it (option Delays).

    comm.tso           = 0.200;  % TSO set-points -> HPPC
    comm.meas_hppc     = 0.020;  % POC and plant measurements -> HPPC
    comm.hppc_to_plant = 0.050;  % HPPC set-points -> plant controllers
    comm.meas_plant    = 0.020;  % asset measurements -> plant controllers
    comm.plant_to_asset = 0.020; % plant controller set-points -> asset controllers

    comm.maxDelay      = 0.5;    % largest delay any case may use [s] (buffer size, compile time)
    comm.delays        = "auto"; % "fixed" (fastest), "variable" (delay sweeps) or "auto", see above
end
