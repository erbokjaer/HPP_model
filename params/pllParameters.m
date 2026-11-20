function pll = pllParameters()
%PLLPARAMETERS  Phase-locked loops (stage 7): one per asset (grid-following
%   current injection) and the frequency measurement of the controllers.
%
%   SRF-PLL on the voltage phasor in the common frame rotating at f0:
%     e   = Im(V e^(-j th)) / |V|  = sin(angle(V) - th)
%     dw  = kp e + ki int(e)       [rad/s], deviation from 2 pi f0
%     th' = dw,  f = 1 + dw / (2 pi f0)  [pu]
%   with kp = 2 zeta wn, ki = wn^2, wn = 2 pi fBW (deriveParameters), so
%   the linearised loop is wn^2 (1 + 2 zeta s/wn) / (s^2 + 2 zeta wn s + wn^2).
%   Prefilter: moving average over Tpre (one cycle, 20 ms) of the voltage
%   in the PLL frame (d, q), as in a moving-average-filter PLL (MAF-PLL).
%   In the RMS (phasor) tier there are no harmonics or unbalance to remove,
%   so the filter only adds its delay (Tpre/2 = 10 ms) inside the loop,
%   which is what limits the bandwidth of a real MAF-PLL. 0 = off.
%
%   asset: the PLL of every asset converter; the current is injected along
%     its angle. on = 0: ideal (the angle of the terminal voltage of the
%     previous step, as before stage 7).
%   hppc, wppc, sppc, bessc: the frequency meter of each controller, its
%     own PLL: the HPPC on the POC voltage, each plant controller on the
%     33 kV busbar voltage. Separate parameters, since in a real plant each
%     controller (vendor) has its own meter. The PLL frequency is averaged
%     over Tavg for control; RoCoF over Trocof (ENTSO-E: 500 ms). on = 0:
%     the system frequency of the grid, unfiltered (as before stage 7).

    pll.asset.on   = 1;      % 1: PLL, 0: ideal (aligned with the terminal voltage)
    pll.asset.fBW  = 5;      % [Hz] natural frequency of the loop (bandwidth)
    pll.asset.zeta = 0.707;  % [-]  damping
    pll.asset.Tpre = 0.02;   % [s]  moving-average prefilter on the phasor (0 = off, max 0.05)
    pll.asset.Tavg = 0;      % [s]  not used by the asset PLL; same fields as the meters, since
    pll.asset.Trocof = 0;    % [s]  PLL.slx takes either as its argument (struct fields must match)

    meter.on     = 1;        % 1: measured (PLL), 0: system frequency
    meter.fBW    = 5;        % [Hz]
    meter.zeta   = 0.707;    % [-]
    meter.Tpre   = 0.02;     % [s]  prefilter (0 = off, max 0.05)
    meter.Tavg   = 0.02;     % [s]  averaging of the PLL frequency for control (max 0.05)
    meter.Trocof = 0.5;      % [s]  RoCoF window (max 1)
    pll.hppc  = meter;       % HPPC, POC voltage
    pll.wppc  = meter;       % wind plant controller, busbar voltage
    pll.sppc  = meter;       % PV plant controller, busbar voltage
    pll.bessc = meter;       % BESS plant controller, busbar voltage
end
