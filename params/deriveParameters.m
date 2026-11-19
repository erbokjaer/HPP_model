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
%     p.pll.asset, .hppc, .wppc, .sppc, .bessc   PLL gains kp, ki and
%                          window lengths in steps

    for t = ["wt", "pv", "bess"]
        p.(t).N      = p.(t).nFeeders * p.(t).nPerFeeder;
        p.(t).Prated = p.(t).N * p.(t).nAgg * p.(t).Pn;
    end

    % communication delays in whole simulation steps (delay lengths of the
    % link blocks; with comm.delays = "variable" the buffers hold up to maxDelay)
    nMax = round(p.comm.maxDelay / p.sim.Ts);
    p.comm.steps = struct();
    for f = setdiff(string(fieldnames(p.comm))', ["maxDelay", "steps", "delays"])
        n = round(p.comm.(f) / p.sim.Ts);
        assert(n >= 0 && n <= nMax, "comm.%s = %g s is outside 0 ... comm.maxDelay = %g s", f, p.comm.(f), p.comm.maxDelay);
        p.comm.steps.(f) = n;
    end

    p.ctrl = controllerParameters(p);

    % PLL gains and window lengths in steps (pllParameters): kp = 2 zeta wn,
    % ki = wn^2; windows as whole steps, within the fixed buffer sizes of
    % the PLL code (pllBody in buildHPP: 50 steps, RoCoF 1000 steps)
    for g = ["asset", "hppc", "wppc", "sppc", "bessc"]
        q = p.pll.(g);
        wn = 2 * pi * q.fBW;
        q.kp = 2 * q.zeta * wn;
        q.ki = wn^2;
        q.Npre = max(1, round(q.Tpre / p.sim.Ts));
        if g ~= "asset"
            q.Navg = max(1, round(q.Tavg / p.sim.Ts));
            q.Nroc = max(1, round(q.Trocof / p.sim.Ts));
        else
            q.Navg = 1;  q.Nroc = 1;
        end
        q.f0 = p.grid.f0;
        q.Ts = p.sim.Ts;
        assert(q.Npre <= 50 && q.Navg <= 50 && q.Nroc <= 1000, ...
            "pll.%s: Tpre and Tavg must be at most 50 steps, Trocof at most 1000 steps", g);
        p.pll.(g) = q;
    end

    % Asset model arguments, one per instance (identical for now)
    p.wt.ap = repmat(struct("ctrl", p.ctrl.wt.par, "gen", p.wt.iec.gen, "pll", p.pll.asset), p.wt.N, 1);
    p.pv.ap = repmat(struct("ctrl", p.ctrl.pv.par, "gen", p.pv.wecc.gen, "pll", p.pll.asset), p.pv.N, 1);

    Eunit = p.bess.E / p.bess.N;                       % [J] per instance
    p.bess.ap = repmat(struct( ...
        "ctrl", p.ctrl.bess.par, ...
        "gen", p.bess.wecc.gen, ...
        "pll", p.pll.asset, ...
        "SOC0", p.bess.SOC0, ...
        "kSOC", p.ctrl.bess.Pbase / Eunit), ...        % [1/s] dSOC/dt per pu power
        p.bess.N, 1);

    % Network solver data (Z matrix etc.), depends on N and ratings
    p.net.solver = networkSolverData(p);
end
