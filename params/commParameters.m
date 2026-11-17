function comm = commParameters()
%COMMPARAMETERS  Communication delays between control levels (typical values).
%   Applied in HPP.slx as a discrete delay of round(T / p.sim.Ts) steps,
%   fixed at compile time; the receiving controller samples the delayed
%   signal at its own rate. All in seconds.
%   Set all to 0 to compare with a model without communication delays.

    comm.tso           = 0.200;  % TSO set-points -> HPPC
    comm.meas_hppc     = 0.020;  % POC and plant measurements -> HPPC
    comm.hppc_to_plant = 0.050;  % HPPC set-points -> plant controllers
    comm.meas_plant    = 0.020;  % asset measurements -> plant controllers
    comm.plant_to_asset = 0.020; % plant controller set-points -> asset controllers
end
