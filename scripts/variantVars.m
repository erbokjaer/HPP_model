function v = variantVars(p, delayCode)
%VARIANTVARS  The variant control variables of a run and where they live.
%
%   v = variantVars(p, delayCode): struct array (name, value, where):
%     HPP_network  network model (networkCode)      HPP model workspace
%     HPP_simscape Simscape mode (simscapeMode:     HPP model workspace
%                  1 phasor, 2 EMT)
%     HPP_delays   delay variant (delayMode)        HPP model workspace
%     HPP_pll      PLL variant (p.pll.type)         global (every PLL
%                                                   variant subsystem reads it)
%   and compile-time values of the Simscape network (HPP model workspace):
%     HPP_Ts       p.sim.Ts (sample time of its clocks and holds)
%     HPP_Tnet     netSolverStep(p): step of its local solver and the
%                  fixed step of HPP.slx
%     HPP_f0       p.grid.f0 (rated frequency of its sources, transformers
%                  and power sensors)
%   These are not taken from p: in Rapid Accelerator, a variable that sets
%   any compile-time parameter is not tunable at all (Simulink ignores its
%   new value), so the network's compile-time parameters must not use p.
%   setVariants puts them on a Simulink.SimulationInput; rapidTarget assigns
%   them for the build; buildTop stores the defaults in the model workspace.

    v = struct("name", {"HPP_network", "HPP_simscape", "HPP_delays", "HPP_pll", "HPP_Ts", "HPP_Tnet", "HPP_f0"}, ...
               "value", {networkCode(p.net.solverType), simscapeMode(p), delayCode, p.pll.type, ...
                         p.sim.Ts, netSolverStep(p), p.grid.f0}, ...
               "where", {"HPP", "HPP", "HPP", "global", "HPP", "HPP", "HPP"});
end
