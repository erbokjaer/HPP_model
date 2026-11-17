function p = deriveParameters(p)
%DERIVEPARAMETERS  Recompute everything that depends on other parameters.
%
%   Called at the end of HPP_parameters and by applyCase after case values
%   are applied, so derived values always follow the component values:
%     p.<type>.N, .Prated  asset instances (nFeeders * nPerFeeder) and plant
%                          rating (N * nAgg * Pn) for wt, pv, bess
%     p.ctrl               all controller parameters (controllerParameters)
%     p.wt.ap, p.pv.ap, p.bess.ap   per-instance arguments of the asset
%                          models (N-by-1 struct arrays); HPP.slx passes
%                          p.<type>.ap(k) to asset instance k

    for t = ["wt", "pv", "bess"]
        p.(t).N      = p.(t).nFeeders * p.(t).nPerFeeder;
        p.(t).Prated = p.(t).N * p.(t).nAgg * p.(t).Pn;
    end

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
