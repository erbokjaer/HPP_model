function s = networkSolverData(p)
%NETWORKSOLVERDATA  Nodal model of collector systems, transformers and grid.
%
%   Positive-sequence phasor model in per unit on the system base
%   p.grid.Pbase (VA) and the nominal voltage of each level (ideal
%   transformer ratios). Layout from p.<type>.nFeeders / nPerFeeder and
%   networkParameters.
%
%   Asset order (global index k): wind, PV, BESS; within a plant feeder by
%   feeder, and along each feeder from the collector bus outwards.
%   Nodes: asset terminals 1..nA, feeder taps nA+1..2nA (MV side of each
%   unit transformer), plant collector buses (plants with a plant
%   transformer), 33 kV HPP busbar (s.bus), POC (s.poc, last).
%   Plant branches (s.br*): the branches through which each plant feeds
%   the HPP busbar; P_plant / Q_plant are the sums of their flows at the
%   busbar end (the plant measurement points).
%
%   The network solves V = Z * (I_assets + I_norton) with Z = inv(Y).
%   Returned as real arrays so it can be a block parameter.

    Sb = p.grid.Pbase;
    w  = 2*pi*p.grid.f0;
    n  = p.net;
    types = ["wt", "pv", "bess"];
    N  = arrayfun(@(t) p.(t).N, types);
    nA = sum(N);
    hasPlantTr = arrayfun(@(t) n.(t).plant.S > 0, types);
    nNodes = 2*nA + sum(hasPlantTr) + 2;
    bus = nNodes - 1;
    poc = nNodes;

    Y = zeros(nNodes);
    plantOf = zeros(nA, 1);
    brFrom = zeros(0, 1);  brZ = zeros(0, 1);  brB = zeros(0, 1);  brPlant = zeros(0, 1);
    nextBus = 2*nA;
    k0 = 0;
    for t = 1:3
        d = n.(types(t));  q = p.(types(t));
        Zb = d.Vcol^2 / Sb;                                   % [ohm] base at the collector voltage
        if hasPlantTr(t)
            nextBus = nextBus + 1;  col = nextBus;
            Zp = impedance(d.plant.uk, d.plant.XR) * Sb / d.plant.S;
            Y = addSeries(Y, col, bus, Zp);
            Y(col, col) = Y(col, col) + magnetizing(d.plant, Sb);   % at the collector side
            brFrom(end+1, 1) = col;  brZ(end+1, 1) = Zp;  brB(end+1, 1) = 0;  brPlant(end+1, 1) = t; %#ok<AGROW>
        else
            col = bus;
        end
        Zt = impedance(d.unit.uk, d.unit.XR) * Sb / (d.unit.S * q.nAgg);
        for f = 1:q.nFeeders
            prev = col;
            for i = 1:q.nPerFeeder
                k = k0 + (f-1)*q.nPerFeeder + i;
                plantOf(k) = t;
                Y = addSeries(Y, k, nA + k, Zt);                 % unit transformer
                if i == 1, L = d.cable.len1; else, L = d.cable.len; end
                Zc = complex(d.cable.R, d.cable.X) * L / d.cable.nPar / Zb;
                Bh = w * d.cable.C * L * d.cable.nPar / 2 * Zb;
                Y = addSeries(Y, prev, nA + k, Zc);              % feeder segment (pi)
                Y(prev, prev) = Y(prev, prev) + 1i*Bh;
                Y(nA+k, nA+k) = Y(nA+k, nA+k) + 1i*Bh;
                if i == 1 && ~hasPlantTr(t)                      % feeder ends on the HPP busbar
                    brFrom(end+1, 1) = nA + k;  brZ(end+1, 1) = Zc;  brB(end+1, 1) = Bh;  brPlant(end+1, 1) = t; %#ok<AGROW>
                end
                prev = nA + k;
            end
        end
        k0 = k0 + N(t);
    end
    % HPP transformer and grid (Ssc = SCR * Pmax_poc)
    Zm = impedance(n.main.uk, n.main.XR) * Sb / n.main.S;
    Y = addSeries(Y, bus, poc, Zm);
    Y(bus, bus) = Y(bus, bus) + magnetizing(n.main, Sb);           % at the 33 kV side
    Zg = impedance(1, p.grid.XR) * Sb / (p.grid.SCR * p.grid.Pmax_poc);
    Y(poc, poc) = Y(poc, poc) + 1/Zg;

    Z = inv(Y);
    s.nA      = nA;
    s.nNodes  = nNodes;
    s.bus     = bus;
    s.poc     = poc;
    s.plantOf = plantOf;              % plant (1 wind, 2 PV, 3 BESS) of each asset
    s.Zre = real(Z);   s.Zim = imag(Z);
    s.brFrom  = brFrom;               % plant branches into the HPP busbar
    s.brZre   = real(brZ);  s.brZim = imag(brZ);
    s.brB     = brB;                  % shunt susceptance at the busbar end
    s.brPlant = brPlant;
    s.Zg_re = real(Zg); s.Zg_im = imag(Zg);
    s.E     = p.grid.E;
    s.Sb    = Sb;
end

function Z = impedance(zmag, XR)
    % complex impedance with magnitude zmag and ratio X/R
    R = zmag / sqrt(1 + XR^2);
    Z = complex(R, R * XR);
end

function y = magnetizing(d, Sb)
    % shunt admittance of a magnetizing branch (Rm, Xm in pu on the
    % transformer rating d.S) in pu on the system base Sb
    y = (1/d.Rm - 1i/d.Xm) * d.S / Sb;
end

function Y = addSeries(Y, a, b, Z)
    y = 1 / Z;
    Y(a, a) = Y(a, a) + y;  Y(b, b) = Y(b, b) + y;
    Y(a, b) = Y(a, b) - y;  Y(b, a) = Y(b, a) - y;
end
