function conv = converterParameters()
%CONVERTERPARAMETERS  Converter part of the assets (work package C, design.md 14.10).
%
%   type: the converter model of all assets, chosen per run (a tunable
%     value: a Switch in each asset's Current injection, no rebuild):
%     1 current source (RMS, the default): I = (ip - j iq) e^(j th_pll);
%     2 averaged converter: inner dq current loop in the PLL frame, LCL
%       filter, simple DC link, angle-preserving current limit. For dynamic
%       networks (Simscape EMT); with the nodal network for comparison.
%
%   Averaged converter, per unit on the asset rating (voltage on the
%   nominal voltage, time in s, reactances at f0). In the common frame at
%   w0 = 2 pi f0, the converter voltage vc, the filter node voltage vn and
%   the terminal voltage vg (from the network, held over the model step):
%     (L1/w0) di1/dt = vc - vn - R1 i1 - j L1 i1
%     (Cf/w0) dvC/dt = i1 - i2 - j Cf vC,    vn = vC + Rd (i1 - i2)
%     (L2/w0) di2/dt = vn - vg - R2 i2 - j L2 i2
%   The network sees the grid-side current i2 (I_re, I_im).
%   Inner loop (PLL frame, d along th_pll): reference i1* = (ip - j iq) +
%   dP / |vn| + j Cf vn (grid current + DC voltage correction + capacitor
%   current), limited to |i1*| <= Imax keeping its angle; PI (kp = aC L1/w0,
%   ki = aC R1, aC = 2 pi fBW: internal model control, loop aC / s) with
%   decoupling j L1 i1 and feed-forward of vn; |vc| <= mMax Vdc (no
%   integration while limited).
%   DC link: stored energy Hdc Vdc^2 (Hdc = C Vdc_n^2 / (2 S_n)),
%     Hdc d(Vdc^2)/dt = Psrc - Re(vc conj(i1)),
%   Psrc = the commanded power Re(vg conj(I*)) through a lag Tsrc (WT
%   machine side, PV DC-DC / MPPT, battery); the grid-side converter holds
%   Vdc (PI on Vdc^2, natural frequency fDC, damping zDC, output dP);
%   a chopper clamps Vdc at Vchop (surplus power while the current limit
%   acts, e.g. in a dip).
%   Time step: the converter runs at the model step p.sim.Ts in one MATLAB
%   Function per asset with n = round(Ts / Tfast) sub-steps (exact
%   zero-order-hold discretization of the filter, deriveParameters); the
%   controllers keep their rates. With an EMT step (Ts <= Tfast) n = 1.

    conv.type = 1;           % 1: current source (RMS), 2: averaged converter

    % LCL filter [pu on the asset rating]; resonance f0 sqrt((L1 + L2) / (L1 L2 Cf))
    conv.L1 = 0.10;          % converter-side inductance (reactance at f0)
    conv.R1 = 0.003;         % its resistance
    conv.Cf = 0.05;          % filter capacitance (susceptance at f0: 5 % reactive power)
    conv.L2 = 0.03;          % grid-side inductance (the unit transformer is in the network)
    conv.R2 = 0.002;         % its resistance
    conv.Rd = NaN;           % passive damping in series with Cf; NaN: 1 / (3 (fres/f0) Cf)

    % inner current loop
    conv.fBW   = 500;        % [Hz] bandwidth aC / 2 pi
    conv.Imax  = 1.2;        % [pu] converter current limit (magnitude of i1*, angle kept)
    conv.mMax  = 1.2;        % [-]  |vc| <= mMax Vdc (modulation limit)
    conv.Tfast = 50e-6;      % [s]  sub-step of the converter and filter (at most the model step)

    % DC link
    conv.Vdc0  = 1;          % [pu] DC voltage reference
    conv.Hdc   = 0.005;      % [s]  stored energy at Vdc0 / rating
    conv.fDC   = 20;         % [Hz] DC voltage loop, natural frequency
    conv.zDC   = 0.7;        % [-]  its damping
    conv.Tsrc  = 0.005;      % [s]  lag of the power source behind the DC link
    conv.Vchop = 1.15;       % [pu] chopper: Vdc clamped here
end
