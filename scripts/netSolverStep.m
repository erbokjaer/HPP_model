function Ts = netSolverStep(p)
%NETSOLVERSTEP  Fundamental step of HPP.slx and step of the Simscape local solver.
%   p.sim.Ts, or p.net.emtTs with the Simscape network in EMT mode (see
%   simscapeMode). HPP.slx uses netSolverStep(p) as its fixed step size.
    [~, Ts] = simscapeMode(p);
end
