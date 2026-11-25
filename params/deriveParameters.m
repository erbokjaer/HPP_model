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
%     p.conv               converter gains, filter discretization (converterGains)

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
        q.w0 = 2 * pi * q.f0;          % [rad/s] for f = 1 + dw / w0 (block expressions: no pi)
        q.wlim = 2 * pi * 5;           % [rad/s] frequency limit of the PLL, +-5 Hz
        assert(q.Npre <= 50 && q.Navg <= 50 && q.Nroc <= 1000, ...
            "pll.%s: Tpre and Tavg must be at most 50 steps, Trocof at most 1000 steps", g);
        p.pll.(g) = q;
    end

    % the asset PLL is part of each asset controller (par.pll)
    for t = ["wt", "pv", "bess"]
        p.ctrl.(t).par.pll = p.pll.asset;
        p.ctrl.(t).par.pll.Ts = p.ctrl.(t).Ts;     % the PLL runs at the controller's rate
    end

    % converter part of the assets (converterParameters): gains and the
    % discretized LCL filter, at the model step
    p.conv = converterGains(p.conv, p.sim.Ts);

    % Asset model arguments, one per instance (identical for now)
    p.wt.ap = repmat(struct("ctrl", p.ctrl.wt.par, "gen", p.wt.iec.gen, "conv", p.conv), p.wt.N, 1);
    p.pv.ap = repmat(struct("ctrl", p.ctrl.pv.par, "gen", p.pv.wecc.gen, "conv", p.conv), p.pv.N, 1);

    Eunit = p.bess.E / p.bess.N;                       % [J] per instance
    p.bess.ap = repmat(struct( ...
        "ctrl", p.ctrl.bess.par, ...
        "gen", p.bess.wecc.gen, ...
        "conv", p.conv, ...
        "SOC0", p.bess.SOC0, ...
        "kSOC", p.ctrl.bess.Pbase / Eunit), ...        % [1/s] dSOC/dt per pu power
        p.bess.N, 1);

    % Network solver data (Z matrix etc.), depends on N and ratings
    p.net.solver = networkSolverData(p);
end

function c = converterGains(c, Ts)
    % derived values of the averaged converter (converterParameters):
    % sub-steps, PI gains of the current and DC voltage loops, damping
    % resistor, and the LCL filter as x[k+1] = Ad x[k] + Bd [vc; vg] with
    % x = [i1; vC; i2] (exact zero-order hold over one sub-step, in the
    % frame at w0), stored as real and imaginary parts
    w0 = 2 * pi * 50;
    c.n = max(1, round(Ts / c.Tfast));
    c.h = Ts / c.n;
    c.fres = 50 * sqrt((c.L1 + c.L2) / (c.L1 * c.L2 * c.Cf));
    if isnan(c.Rd), c.RdUsed = 1 / (3 * (c.fres / 50) * c.Cf); else, c.RdUsed = c.Rd; end
    aC = 2 * pi * c.fBW;
    c.kp = aC * c.L1 / w0;
    c.ki = aC * c.R1;
    wDC = 2 * pi * c.fDC;
    c.kpDC = 2 * c.zDC * wDC * c.Hdc;
    c.kiDC = wDC^2 * c.Hdc;
    Rd = c.RdUsed;
    A = [-w0 * (c.R1 + Rd) / c.L1 - 1j * w0,  -w0 / c.L1,   w0 * Rd / c.L1
          w0 / c.Cf,                          -1j * w0,     -w0 / c.Cf
          w0 * Rd / c.L2,                      w0 / c.L2,   -w0 * (c.R2 + Rd) / c.L2 - 1j * w0];
    B = [w0 / c.L1, 0;  0, 0;  0, -w0 / c.L2];
    M = expm([A, B; zeros(2, 5)] * c.h);
    c.AdRe = real(M(1:3, 1:3));  c.AdIm = imag(M(1:3, 1:3));
    c.BdRe = real(M(1:3, 4:5));  c.BdIm = imag(M(1:3, 4:5));
end
