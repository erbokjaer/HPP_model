function s = networkDqData(p)
%NETWORKDQDATA  Linear state-space model of the network in the dq frame (stage 8).
%
%   The same collector systems, transformers and grid as networkSolverData
%   (positive sequence, per unit on p.grid.Pbase), but with the network's
%   electromagnetic dynamics, in the frame rotating at w0 = 2 pi f0 and
%   written with complex quantities (d + j q):
%     branches  L di/dt = v_from - v_to - (R + j w0 L) i
%     nodes     C dv/dt = sum(i_in) - (G + j w0 C) v - i_mag + I_inj
%     magnetizing inductors  Lm dim/dt = v - j w0 Lm im
%   with L = X / w0, C = B / w0 (X, B in pu at f0). In steady state this is
%   exactly the nodal model.
%
%   Nodes with states: feeder taps (MV side of each unit transformer, one
%   per asset), plant collector buses, the 33 kV HPP busbar; all have cable
%   capacitance. Asset terminals and the POC have none (only inductive
%   branches meet there), so they are not states:
%   - each asset's current flows through its unit transformer unchanged:
%     I is injected at the feeder tap, and the terminal voltage is
%     v_term = v_tap + Zt I (quasi-static: L dI/dt of the unit transformer
%     left out, as in the RMS asset models that inject I directly);
%   - the HPP transformer and the grid impedance form one branch from the
%     busbar to the grid source E e^(j theta); the POC voltage follows from
%     its current and derivative exactly (divider of the two R-L parts).
%
%   Discretization: exact zero-order hold at p.sim.Ts (matrix exponential),
%   x[k+1] = Ad x[k] + Bd u[k], for inputs held over a step, so no inner
%   sub-steps are needed. Outputs y = Cy x + Dy u, evaluated with the state
%   at the end of the step: [v_term (nA); v_bus; v_poc; I_grid (into the
%   grid at the POC); I_br (plant branches into the busbar, incl. charging
%   at the busbar end)]. Real
%   form: complex vectors z are stored as [real(z); imag(z)]; inputs
%   u = [I (nA); E e^(j theta)]. X0 = -A \ B gives the steady state for an
%   input (initial state, no start-up transient).

    Sb = p.grid.Pbase;
    w0 = 2*pi*p.grid.f0;
    Ts = p.sim.Ts;
    n  = p.net;
    types = ["wt", "pv", "bess"];
    N  = arrayfun(@(t) p.(t).N, types);
    nA = sum(N);
    hasPlantTr = arrayfun(@(t) n.(t).plant.S > 0, types);
    nN = nA + sum(hasPlantTr) + 1;          % taps, collector buses, busbar
    bus = nN;

    % element lists (complex impedances in pu on Sb)
    br = struct("from", {}, "to", {}, "Z", {});     % to = 0: grid source
    Cn = zeros(nN, 1);  Gn = zeros(nN, 1);           % node capacitance (B), conductance
    mag = struct("node", {}, "X", {});
    Zu = zeros(nA, 1);                              % unit transformer per asset
    plantBr = zeros(0, 1);  plantBrB = zeros(0, 1);  plantOfBr = zeros(0, 1);
    nextCol = nA;  k0 = 0;
    for t = 1:3
        d = n.(types(t));  q = p.(types(t));
        Zb = d.Vcol^2 / Sb;
        if hasPlantTr(t)
            nextCol = nextCol + 1;  col = nextCol;
            Zp = impedance(d.plant.uk, d.plant.XR) * Sb / d.plant.S;
            br(end+1) = struct("from", col, "to", bus, "Z", Zp); %#ok<AGROW>
            plantBr(end+1, 1) = numel(br);  plantBrB(end+1, 1) = 0;  plantOfBr(end+1, 1) = t; %#ok<AGROW>
            [g, xm] = magnetizing(d.plant, Sb);
            Gn(col) = Gn(col) + g;  mag(end+1) = struct("node", col, "X", xm); %#ok<AGROW>
        else
            col = bus;
        end
        Zt = impedance(d.unit.uk, d.unit.XR) * Sb / (d.unit.S * q.nAgg);
        for f = 1:q.nFeeders
            prev = col;
            for i = 1:q.nPerFeeder
                k = k0 + (f-1)*q.nPerFeeder + i;
                Zu(k) = Zt;
                if i == 1, L = d.cable.len1; else, L = d.cable.len; end
                Zc = complex(d.cable.R, d.cable.X) * L / d.cable.nPar / Zb;
                Bh = w0 * d.cable.C * L * d.cable.nPar / 2 * Zb;
                br(end+1) = struct("from", k, "to", prev, "Z", Zc); %#ok<AGROW>  % tap -> inner node
                Cn(k) = Cn(k) + Bh;  Cn(prev) = Cn(prev) + Bh;
                if i == 1 && ~hasPlantTr(t)                  % feeder ends on the HPP busbar
                    plantBr(end+1, 1) = numel(br);  plantBrB(end+1, 1) = Bh;  plantOfBr(end+1, 1) = t; %#ok<AGROW>
                end
                prev = k;
            end
        end
        k0 = k0 + N(t);
    end
    % HPP transformer + grid impedance: one branch busbar -> grid source
    Zm = impedance(n.main.uk, n.main.XR) * Sb / n.main.S;
    Zg = impedance(1, p.grid.XR) * Sb / (p.grid.SCR * p.grid.Pmax_poc);
    br(end+1) = struct("from", bus, "to", 0, "Z", Zm + Zg);
    gBr = numel(br);
    [g, xm] = magnetizing(n.main, Sb);
    Gn(bus) = Gn(bus) + g;  mag(end+1) = struct("node", bus, "X", xm);
    assert(all(Cn > 0), "networkDqData: every state node needs cable capacitance (a cable of length 0?)");

    % ---- continuous complex model: x = [v (nN); i (nB); im (nM)], u = [I (nA); E] ----
    nB = numel(br);  nM = numel(mag);  nx = nN + nB + nM;  nu = nA + 1;
    C = Cn / w0;                                       % [pu s]
    A = complex(zeros(nx));  B = complex(zeros(nx, nu));
    iv = @(nd) nd;  ii = @(b) nN + b;  im = @(m) nN + nB + m;
    for nd = 1:nN
        A(iv(nd), iv(nd)) = -(Gn(nd) + 1i*w0*C(nd)) / C(nd);
    end
    for b = 1:nB
        Lb = imag(br(b).Z) / w0;  Rb = real(br(b).Z);
        a = br(b).from;  c = br(b).to;
        A(ii(b), ii(b)) = -(Rb + 1i*w0*Lb) / Lb;
        A(ii(b), iv(a)) = 1 / Lb;
        A(iv(a), ii(b)) = A(iv(a), ii(b)) - 1 / C(a);      % leaves its from node
        if c > 0
            A(ii(b), iv(c)) = -1 / Lb;
            A(iv(c), ii(b)) = A(iv(c), ii(b)) + 1 / C(c);  % enters its to node
        else
            B(ii(b), nu) = -1 / Lb;                       % grid source E
        end
    end
    for m = 1:nM
        Lm = mag(m).X / w0;  nd = mag(m).node;
        A(im(m), im(m)) = -1i*w0;
        A(im(m), iv(nd)) = 1 / Lm;
        A(iv(nd), im(m)) = -1 / C(nd);
    end
    for k = 1:nA
        B(iv(k), k) = 1 / C(k);                            % asset current at its tap
    end

    % ---- outputs y = Cy x + Dy u: v_term (nA), v_bus, v_poc, I_grid, I_br (plant branches) ----
    nBr = numel(plantBr);  ny = nA + 3 + nBr;
    Cy = complex(zeros(ny, nx));  Dy = complex(zeros(ny, nu));
    for k = 1:nA
        Cy(k, iv(k)) = 1;  Dy(k, k) = Zu(k);
    end
    Cy(nA + 1, iv(bus)) = 1;
    % v_poc = E + Zg i + Lg di/dt, di/dt from the branch equation (exact)
    % with Lt di/dt = v_bus - E - (Rt + j w0 Lt) i (the branch equation) and
    % Lg di/dt = kL * Lt di/dt, kL = Lg / Lt
    Lt = imag(Zm + Zg) / w0;  Lg = imag(Zg) / w0;  kL = Lg / Lt;
    rowDi = Lt * A(ii(gBr), :);                            % Lt di/dt, state part
    Cy(nA + 2, :) = kL * rowDi;
    Cy(nA + 2, ii(gBr)) = Cy(nA + 2, ii(gBr)) + Zg;
    Dy(nA + 2, nu) = 1 + kL * Lt * B(ii(gBr), nu);         % E - kL E
    Cy(nA + 3, ii(gBr)) = 1;                               % current into the grid (through the POC)
    for j = 1:nBr
        Cy(nA + 3 + j, ii(plantBr(j))) = 1;
        Cy(nA + 3 + j, iv(bus)) = -1i * plantBrB(j);       % minus the charging at the busbar end
    end

    % ---- exact ZOH discretization at Ts (augmented matrix exponential) ----
    Ar = realify(A);  Br = realify(B);
    M = expm([Ar, Br; zeros(2*nu, 2*nx + 2*nu)] * Ts);
    s.Ad = M(1:2*nx, 1:2*nx);
    s.Bd = M(1:2*nx, 2*nx+1:end);
    s.X0 = -(Ar \ Br);                                     % steady state for a held input
    s.Cy = realify(Cy);
    s.Dy = realify(Dy);
    s.nA = nA;  s.nx = nx;  s.ny = ny;  s.nBr = nBr;
    s.brPlant = plantOfBr;                                  % plant of each plant branch
    s.E  = p.grid.E;
    s.Sb = Sb;
    % fastest network mode (information): highest |eigenvalue| frequency [Hz]
    s.fmax = max(abs(imag(eig(A)))) / (2*pi);
end

function Z = impedance(zmag, XR)
    R = zmag / sqrt(1 + XR^2);
    Z = complex(R, R * XR);
end

function [g, x] = magnetizing(d, Sb)
    % magnetizing branch in pu on Sb: conductance g (shunt) and reactance x
    g = d.S / (Sb * d.Rm);
    x = d.Xm * Sb / d.S;
end

function R = realify(M)
    % complex linear map -> real map on [real; imag]
    R = [real(M), -imag(M); imag(M), real(M)];
end
