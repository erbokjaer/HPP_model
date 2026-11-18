function pc = plantControlParameters()
%PLANTCONTROLPARAMETERS  WECC REPC_A settings of the plant controllers.
%
%   pc.wppc / pc.sppc / pc.bessc, copied into p.ctrl.<plant>.par by
%   controllerParameters. Each plant controller closes its own loops (P
%   with frequency response, Q or V) on its feeder into the 33 kV busbar,
%   as vendor plant controllers do; the HPPC is the outer loop at the POC.
%   Override per case, e.g. {"pc.wppc.Frqflag", 0}.
%   The WPPC uses the same REPC_A structure for now (the IEC 61400-27-1
%   WPP control model has the same functions; its naming and details are
%   a later step).

    r = repcDefaults();
    pc.wppc  = r;
    pc.sppc  = r;
    pc.bessc = r;
    % wind and PV run at available power: no upward frequency response
    pc.wppc.Dup = 0;
    pc.sppc.Dup = 0;
end

function r = repcDefaults()
    % WECC REPC_A plant controller (closed loops on the plant's own feeder
    % into the 33 kV busbar). Names after the WECC specification; values
    % typical / placeholder. pu on the plant rating, times in s. Deviation
    % from REPC_A: the P and Q references are fed forward around the PI
    % controllers (common in vendor plant controllers), so the PI only
    % corrects the remaining error (losses, asset deviations).
    r.Tfltr   = 0.02;    % voltage and reactive power measurement filter
    r.RefFlag = 0;       % 0: Q control (track Q_set), 1: voltage control (Vref, droop Kc)
    r.Vref    = 1.0;     % voltage reference (RefFlag = 1)
    r.Kc      = 0.0;     % reactive droop for voltage control [pu V / pu Q]
    r.dbd1    = 0;       % deadband on the V / Q error, lower
    r.dbd2    = 0;       % deadband on the V / Q error, upper
    r.emax    = 0.1;     % max V / Q error
    r.emin    = -0.1;    % min V / Q error
    r.Kp      = 0.1;     % Q / V PI, proportional gain
    r.Ki      = 1.0;     % Q / V PI, integral gain [1/s]
    r.Vfrz    = 0.7;     % Q / V PI frozen while V < Vfrz (e.g. in a dip)
    r.Qmax    = 0.5;     % plant Q order limits
    r.Qmin    = -0.5;
    r.Tfv     = 0.05;    % Q order lag (REPC_A lead-lag with Tft = 0)
    r.Tp      = 0.02;    % active power measurement filter
    r.Frqflag = 0;       % 1: closed-loop P control and frequency response (decentral),
                         % 0: P_set passed on (central control: the HPPC closes the P loop
                         %    at the POC and gives the frequency response, hppcParameters)
    r.Freq_ref = 1.0;    % frequency reference [pu]
    r.fdbd1   = -0.0004; % frequency deadband, lower [pu] (-20 mHz)
    r.fdbd2   = 0.0004;  % frequency deadband, upper [pu]
    r.Ddn     = 20;      % droop for over-frequency [pu P / pu f] (5 %)
    r.Dup     = 20;      % droop for under-frequency
    r.femax   = 0.5;     % max P error
    r.femin   = -0.5;    % min P error
    r.Kpg     = 0.1;     % P PI, proportional gain
    r.Kig     = 1.0;     % P PI, integral gain [1/s]
    r.Tlag    = 0.1;     % P order lag
end
