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

    % Voltage control (hppc.RefFlag = 1). With hppc.Vlocal = 1 the plant
    % controllers control the 33 kV busbar voltage themselves (fast), on a
    % V-Q droop line Q = Q_set + (Vref - V)/Kc whose offset Q_set comes from
    % the HPPC; the HPPC only trims the offsets slowly so that V and Q at the
    % POC lie on the required slope. The busbar droop (all plants together,
    % pu on Pbase) is the POC slope minus the 33/132 kV transformer
    % reactance, so the plants alone already give about the POC slope; it is
    % kept >= KcBusMin, since a negative droop makes the parallel plants
    % unstable against each other (circulating Q).
    Sb    = p.grid.Pbase;
    tr    = p.net.main;
    xTr   = tr.uk * tr.XR / hypot(1, tr.XR) * Sb / tr.S;     % [pu on Pbase]
    xBus  = 1 / scr + xTr;                                    % busbar dV/dQ [pu on Pbase]
    KcPoc = p.hppc.Vslope / p.hppc.Qmax;                      % required slope at the POC
    KcBus = max(KcPoc - xTr, p.hppc.KcBusMin);                % busbar droop, all plants
    sumP  = sum(Prated) / Sb;
    vlocal = p.hppc.RefFlag == 1 && p.hppc.Vlocal == 1;
    % POC Q limit with local voltage control: held by the HPPC (block "POC Q
    % limit"). The plants keep their own order limits: those limit the Q at
    % the asset terminals, and the wind plant loses up to ~40 % of it in its
    % transformers and cables, so a tighter order limit would hold the plant
    % below its share at the busbar.
    for c = ["wppc", "sppc", "bessc"]
        % on each plant's own rating: equal droop k makes the Q split follow the ratings
        ctrl.(c).par.dVdQ = xBus * sumP;
        if vlocal
            ctrl.(c).par.RefFlag = 1;
            ctrl.(c).par.Vref    = p.hppc.Vref;
            ctrl.(c).par.Kc      = KcBus * sumP;
        end
    end
    if vlocal   % HPPC: slow trim of the plants' Q offsets
        auth = KcBus / (KcBus + xBus);       % share of an offset change that becomes real Q
        ctrl.hppc.par.LV   = (KcPoc + 1 / scr) * auth;
        ctrl.hppc.par.KpVn = p.hppc.KpVt;
        ctrl.hppc.par.KiVn = p.hppc.KiVt;
        ctrl.hppc.par.QmaxOrd = p.hppc.Qmax / auth;   % offset range; real Q limited by the plants
        ctrl.hppc.par.QminOrd = p.hppc.Qmin / auth;
    else        % HPPC: direct voltage control at the POC
        ctrl.hppc.par.LV   = KcPoc + 1 / scr;
        ctrl.hppc.par.KpVn = p.hppc.KpV;
        ctrl.hppc.par.KiVn = p.hppc.KiV;
        ctrl.hppc.par.QmaxOrd = p.hppc.Qmax;
        ctrl.hppc.par.QminOrd = p.hppc.Qmin;
    end
    % Derived values for block parameters: blocks that use model arguments
    % may only use + - * / to stay tunable in generated code (accelerator,
    % DLL), so flags, tan(phi) and the mode-dependent Q PI gains are computed here
    ctrl.hppc.par.isV    = double(p.hppc.RefFlag == 1);          % voltage control
    ctrl.hppc.par.isPF   = double(p.hppc.RefFlag == 2);          % power factor control
    ctrl.hppc.par.tanphi = p.hppc.PFsign * sqrt(1 / p.hppc.PF^2 - 1);
    if p.hppc.RefFlag == 1
        ctrl.hppc.par.KpQ = ctrl.hppc.par.KpVn / ctrl.hppc.par.LV;
        ctrl.hppc.par.KiQ = ctrl.hppc.par.KiVn / ctrl.hppc.par.LV;
    else
        ctrl.hppc.par.KpQ = p.hppc.Kpq;
        ctrl.hppc.par.KiQ = p.hppc.Kiq;
    end

    % Cascade check: every level must be clearly slower than the one under
    % it. Speed = closed-loop time constant of the PI, about (1 + Kp)/Ki with
    % the gains normalised by the loop gain (Q loops: loop gain 1).
    tau = @(kp, ki) (1 + kp) / ki;
    tAsset = max([tau(p.wt.iec.ctrl.kpq, p.wt.iec.ctrl.kiq), tau(p.pv.wecc.ctrl.Kqp, p.pv.wecc.ctrl.Kqi), ...
                  tau(p.bess.wecc.ctrl.Kqp, p.bess.wecc.ctrl.Kqi)]);
    pcs = [p.pc.wppc, p.pc.sppc, p.pc.bessc];
    if vlocal
        tPlant = max(arrayfun(@(r) tau(r.KpV, r.KiV), pcs));  plantLoop = "plant voltage";
        tHppc  = tau(p.hppc.KpVt, p.hppc.KiVt);               hppcLoop  = "HPPC voltage trim";
    else
        tPlant = max(arrayfun(@(r) tau(r.Kp, r.Ki), pcs));    plantLoop = "plant Q";
        if p.hppc.RefFlag == 1
            tHppc = tau(p.hppc.KpV, p.hppc.KiV);              hppcLoop  = "HPPC voltage";
        else
            tHppc = tau(p.hppc.Kpq, p.hppc.Kiq);              hppcLoop  = "HPPC Q / PF";
        end
    end
    ctrl.hppc.cascade = struct("asset", tAsset, "plant", tPlant, "hppc", tHppc, ...
        "plantLoop", plantLoop, "hppcLoop", hppcLoop);
    sep = p.hppc.cascadeSep;
    if tPlant < sep * tAsset || tHppc < sep * tPlant
        warning("controllerParameters:cascade", ...
            "Control cascade not separated by %gx: asset Q %.2f s, %s %.2f s, %s %.2f s.", ...
            sep, tAsset, plantLoop, tPlant, hppcLoop, tHppc);
    end

    % Asset controllers (run at the simulation step)
    ctrl.wt   = assetCtrl(p.sim.Ts, p.wt.nAgg * p.wt.Pn);
    for f = string(fieldnames(p.wt.iec.ctrl))'     % IEC 61400-27-1 Type 4A control
        ctrl.wt.par.(f) = p.wt.iec.ctrl.(f);
    end
    ctrl.wt.par.MqG1 = double(p.wt.iec.ctrl.MqG == 1);   % closed-loop Q flag (block parameter, see above)
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
