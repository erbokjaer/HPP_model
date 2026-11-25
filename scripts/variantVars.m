function v = variantVars(p, delayCode)
%VARIANTVARS  The variant control variables of a run and where they live.
%
%   v = variantVars(p, delayCode): struct array (name, value, where):
%     HPP_network  network model (networkCode)      HPP model workspace
%     HPP_delays   delay variant (delayMode)        HPP model workspace
%     HPP_pll      PLL variant (p.pll.type)         global (every PLL
%                                                   variant subsystem reads it)
%   setVariants puts them on a Simulink.SimulationInput; rapidTarget assigns
%   them for the build.

    v = struct("name", {"HPP_network", "HPP_delays", "HPP_pll"}, ...
               "value", {networkCode(p.net.solverType), delayCode, p.pll.type}, ...
               "where", {"HPP", "HPP", "global"});
end
