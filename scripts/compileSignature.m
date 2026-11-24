function sig = compileSignature(p)
%COMPILESIGNATURE  The values of p that are fixed when HPP.slx is compiled.
%
%   The layout (number of units), the time step, the delay mode with the
%   delays (fixed) or the size of the delay buffers (variable), and the
%   variant choices: network model and PLL. All other values are tunable
%   (network data, controllers, grid events). A Fast Restart model or a
%   Rapid Accelerator target is reused only while this stays the same
%   (runCase, rapidTarget); runSweep's checkTunable refuses case sets that
%   change these columns.

    sig = struct("N", [p.wt.N, p.pv.N, p.bess.N], "Ts", p.sim.Ts, ...
        "delays", delaySignature(p), "network", p.net.solverType, "pll", p.pll.type);
end
