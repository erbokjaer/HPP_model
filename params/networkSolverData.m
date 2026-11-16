function s = networkSolverData(p)
%NETWORKSOLVERDATA  Nodal model of collector system, transformers and grid.
%
%   Positive-sequence phasor model in per unit on the system base
%   p.grid.Pbase (VA) and the nominal voltage of each level.
%
%   Asset order (global index): wind 1..Nwt, PV, BESS.
%   Nodes:  asset terminals 1..nA, feeder starts nA+1..2nA (MV side of the
%           unit transformers), MV busbar 2nA+1, POC 2nA+2.
%   Branches: unit transformer (series), feeder cable (pi), main
%   transformer (series), grid (Norton equivalent at the POC).
%
%   The network solves V = Z * (I_assets + I_norton) with Z = inv(Y).
%   Returned as real matrices so it can be a block parameter.

    Sb = p.grid.Pbase;
    w  = 2*pi*p.grid.f0;
    n  = p.net;

    types  = ["wt", "pv", "bess"];
    Srated = [p.ctrl.wt.Pbase, p.ctrl.pv.Pbase, p.ctrl.bess.Pbase];   % instance ratings
    N      = [p.wt.N, p.pv.N, p.bess.N];
    nA = sum(N);
    nNodes = 2*nA + 2;
    bus = 2*nA + 1;
    poc = 2*nA + 2;

    Y = zeros(nNodes);
    Zcab = zeros(nA, 1); Bhalf = zeros(nA, 1); plantOf = zeros(nA, 1);
    Zbase_mv = n.Vmv^2 / Sb;
    k = 0;
    for t = 1:3
        for i = 1:N(t)
            k = k + 1;
            plantOf(k) = t;
            % unit transformer, uk on the instance rating
            Zt = impedance(n.unit.uk, n.unit.XR) * Sb / Srated(t);
            Y = addSeries(Y, k, nA + k, Zt);
            % feeder cable, pi model
            L  = n.cable.len.(types(t));
            np = n.cable.nPar.(types(t));
            Zc = complex(n.cable.R, n.cable.X) * L / np / Zbase_mv;
            Bh = w * n.cable.C * L * np / 2 * Zbase_mv;
            Y = addSeries(Y, nA + k, bus, Zc);
            Y(nA + k, nA + k) = Y(nA + k, nA + k) + 1i*Bh;
            Y(bus, bus)       = Y(bus, bus)       + 1i*Bh;
            Zcab(k) = Zc; Bhalf(k) = Bh;
        end
    end
    % main transformer
    Zm = impedance(n.main.uk, n.main.XR) * Sb / n.main.S;
    Y = addSeries(Y, bus, poc, Zm);
    % grid: Ssc = SCR * Pmax_poc
    Zg = impedance(1, p.grid.XR) * Sb / (p.grid.SCR * p.grid.Pmax_poc);
    Y(poc, poc) = Y(poc, poc) + 1/Zg;

    Z = inv(Y);
    s.nA      = nA;
    s.nNodes  = nNodes;
    s.bus     = bus;
    s.poc     = poc;
    s.plantOf = plantOf;              % plant (1 wind, 2 PV, 3 BESS) of each asset
    s.Zre = real(Z);   s.Zim = imag(Z);
    s.Zcab_re = real(Zcab); s.Zcab_im = imag(Zcab);
    s.Bhalf = Bhalf;
    s.Zg_re = real(Zg); s.Zg_im = imag(Zg);
    s.E     = p.grid.E;
    s.Sb    = Sb;
end

function Z = impedance(zmag, XR)
    % complex impedance with magnitude zmag and ratio X/R
    R = zmag / sqrt(1 + XR^2);
    Z = complex(R, R * XR);
end

function Y = addSeries(Y, a, b, Z)
    y = 1 / Z;
    Y(a, a) = Y(a, a) + y;  Y(b, b) = Y(b, b) + y;
    Y(a, b) = Y(a, b) - y;  Y(b, a) = Y(b, a) - y;
end
