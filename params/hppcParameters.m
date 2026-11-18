function hppc = hppcParameters()
%HPPCPARAMETERS  Settings of the HPP controller (HPPC), stage 5.
%
%   Copied into p.ctrl.hppc.par by controllerParameters. Per unit on the
%   POC base p.grid.Pbase (180 MW), frequency in pu of 50 Hz, times in s.
%
%   Central control (default): the HPPC closes the P loop at the POC and
%   gives the frequency response; the plant controllers pass their P
%   set-points on (p.pc.<plant>.Frqflag = 0: no local droop, no plant P
%   loop). Their Q loops stay closed.
%   Decentral (stage-4 behaviour, for comparison): Pflag = 0, Frqflag = 0
%   here and Frqflag = 1 in the plant controllers.
%
%   Override per case, e.g. {"hppc.Frqflag", 0}.

    hppc.Pflag   = 1;        % 1: closed-loop P at the POC (PI on P_target - P_poc), 0: open loop
    hppc.Frqflag = 1;        % 1: frequency response in the HPPC (central), 0: none
    hppc.Tp      = 0.2;      % P_poc measurement filter (>= Ts = 0.1 s)
    hppc.Tref    = 0.4;      % reference filter on the target in the PI error: the PI
                             % corrects only what the plant does not follow (losses,
                             % deviations), not the response time of a step
                             % (measurement filter, delays, plant lags)
    hppc.Kpg     = 0.1;      % P PI, proportional gain [pu / pu]
    hppc.Kig     = 0.5;      % P PI, integral gain [1/s]
    hppc.Vfrz    = 0.9;      % P PI frozen while V_poc < Vfrz (voltage dip) ...
    hppc.Thld    = 1.0;      % ... and for Thld after it (assets recover their P)
    hppc.femax   = 0.2;      % max P error
    hppc.femin   = -0.2;     % min P error

    % frequency response, droop on the POC base (5 %: 20 pu P / pu f,
    % 0.072 pu = 13 MW for +0.2 Hz outside the deadband)
    hppc.Freq_ref = 1.0;     % frequency reference [pu]
    hppc.fdbd1   = -0.0004;  % deadband, lower [pu] (-20 mHz)
    hppc.fdbd2   = 0.0004;   % deadband, upper [pu]
    hppc.Ddn     = 20;       % droop for over-frequency [pu P / pu f]
    hppc.Dup     = 20;       % droop for under-frequency (headroom: BESS)
end
