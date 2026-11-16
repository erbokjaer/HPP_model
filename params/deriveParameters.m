function p = deriveParameters(p)
%DERIVEPARAMETERS  Recompute everything that depends on other parameters.
%
%   Called at the end of HPP_parameters and by applyCase after case values
%   are applied, so derived values always follow the component values:
%     p.wt.Prated          plant rating from N, nAgg and Pn
%     p.ctrl               all controller parameters (controllerParameters)
%     p.wt.ap, p.pv.ap, p.bess.ap   per-instance arguments of the asset
%                          models (N-by-1 struct arrays); HPP.slx passes
%                          p.<type>.ap(k) to asset instance k

    p.wt.Prated = p.wt.N * p.wt.nAgg * p.wt.Pn;

    p.ctrl = controllerParameters(p);

    % Asset model arguments, one per instance (identical for now)
    p.wt.ap = repmat(struct("ctrl", p.ctrl.wt.par, "gen", p.wt.iec.gen), p.wt.N, 1);
    p.pv.ap = repmat(struct("ctrl", p.ctrl.pv.par, "gen", p.pv.wecc.gen), p.pv.N, 1);

    Eunit = p.bess.E / p.bess.N;                       % [J] per instance
    p.bess.ap = repmat(struct( ...
        "ctrl", p.ctrl.bess.par, ...
        "gen", p.bess.wecc.gen, ...
        "SOC0", p.bess.SOC0, ...
        "kSOC", p.ctrl.bess.Pbase / Eunit), ...        % [1/s] dSOC/dt per pu power
        p.bess.N, 1);

    % Network solver data (Z matrix etc.), depends on N and ratings
    p.net.solver = networkSolverData(p);
end
