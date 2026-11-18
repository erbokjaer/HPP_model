function ctrl = controllerParameters(p)
%CONTROLLERPARAMETERS  Parameters of all controllers in the hierarchy.
%
%   ctrl.hppc                      HPP controller (POC)
%   ctrl.wppc / ctrl.sppc / ctrl.bessc   plant controllers (wind, PV, BESS)
%   ctrl.wt / ctrl.pv / ctrl.bess  asset controllers (one set for all N
%                                  instances of a type)
%
%   Every controller has the same three fields:
%     Ts     fixed sample time [s]
%     Pbase  power base of its per-unit interface [W]: POC base for the
%            HPPC, plant rating for plant controllers, asset (instance)
%            rating for asset controllers
%     par    tunable parameters, in pu on Pbase. Passed to the controller
%            model as the model argument "par"; becomes the parameter
%            struct of the generated code / IEEE-CIGRE DLL.
%   Ts and Pbase are fixed settings. Conversions between W and pu are done
%   in HPP.slx, outside the controllers. See docs/design.md.
%
%   Called by deriveParameters, after the component parameters are set.

    % HPP controller
    ctrl.hppc.Ts    = 0.1;
    ctrl.hppc.Pbase = p.grid.Pbase;
    ctrl.hppc.par.Ppoc_max = p.grid.Pmax_poc / ctrl.hppc.Pbase;   % [pu] (not used yet)
    Prated = [p.wt.Prated; p.pv.Prated; p.bess.Prated];
    ctrl.hppc.par.qshare = Prated / sum(Prated);                  % [-] Q_ref split wind/PV/BESS
    ctrl.hppc.par = copyFields(ctrl.hppc.par, p.hppc);            % P loop, frequency response (hppcParameters)
    scr = p.hppc.SCRset;
    if isnan(scr), scr = p.grid.SCR; end                           % commissioned for the actual grid
    ctrl.hppc.par.dVdQ = 1 / scr;                                  % voltage-control loop gain of the grid
    ctrl.hppc.par.SOCmin = p.bess.SOCmin;                          % for the SOC-aware BESS limits
    ctrl.hppc.par.SOCmax = p.bess.SOCmax;
    % grid code ranges (Energinet DK1): ramp 1-20 %/min, at most 60 MW/min; droops 2-12 %
    rampMax = min(0.20, 60e6 / p.grid.Pbase);
    assert(p.hppc.rampUp >= 0.01 && p.hppc.rampUp <= rampMax && p.hppc.rampDn >= 0.01 && ...
        p.hppc.rampDn <= rampMax, "hppc.rampUp / rampDn must be within 1-20 %% of Pn per minute (max 60 MW/min)");
    assert(all([p.hppc.sO p.hppc.sU p.hppc.sFSM] >= 0.02 & [p.hppc.sO p.hppc.sU p.hppc.sFSM] <= 0.12), ...
        "hppc droops must be within 2-12 %%");

    % Plant controllers: equal share per asset instance
    ctrl.wppc  = plantCtrl(p.wt.Prated,   p.wt.N);
    ctrl.sppc  = plantCtrl(p.pv.Prated,   p.pv.N);
    ctrl.bessc = plantCtrl(p.bess.Prated, p.bess.N);
    % WECC REPC_A settings of each plant controller (p.pc, plantControlParameters)
    for c = ["wppc", "sppc", "bessc"]
        ctrl.(c).par = copyFields(ctrl.(c).par, p.pc.(c));
    end
    ctrl.bessc.par.SOCmin = p.bess.SOCmin;
    ctrl.bessc.par.SOCmax = p.bess.SOCmax;

    % Asset controllers (run at the simulation step)
    ctrl.wt   = assetCtrl(p.sim.Ts, p.wt.nAgg * p.wt.Pn);
    for f = string(fieldnames(p.wt.iec.ctrl))'     % IEC 61400-27-1 Type 4A control
        ctrl.wt.par.(f) = p.wt.iec.ctrl.(f);
    end
    ctrl.pv   = assetCtrl(p.sim.Ts, p.pv.nAgg * p.pv.Pn);
    ctrl.pv.par = copyFields(ctrl.pv.par, p.pv.wecc.ctrl);         % WECC REEC_B
    ctrl.bess = assetCtrl(p.sim.Ts, p.bess.nAgg * p.bess.Pn);
    ctrl.bess.par.Pmax = (p.bess.Pmax / p.bess.N) / ctrl.bess.Pbase;  % [pu] power limit
    ctrl.bess.par = copyFields(ctrl.bess.par, p.bess.wecc.ctrl);     % WECC REEC_C
    ctrl.bess.par.SOCmin = p.bess.SOCmin;
    ctrl.bess.par.SOCmax = p.bess.SOCmax;

    for c = string(fieldnames(ctrl))'
        r = ctrl.(c).Ts / p.sim.Ts;
        assert(abs(r - round(r)) < 1e-9, "ctrl.%s.Ts must be an integer multiple of sim.Ts", c);
    end
end

function a = copyFields(a, b)
    for f = string(fieldnames(b))'
        a.(f) = b.(f);
    end
end

function c = plantCtrl(Prated, N)
    c.Ts    = 0.02;
    c.Pbase = Prated;
    c.par.share = ones(N, 1) / N;   % [-] fallback split when nothing is available
end

function c = assetCtrl(Ts, Pbase)
    c.Ts    = Ts;
    c.Pbase = Pbase;
    c.par.Vmin = 0.01;              % [pu] voltage floor for current commands
end
