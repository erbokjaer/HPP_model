function buildHPP(opts)
%BUILDHPP  (Re)build the hierarchical HPP model from the parameters.
%
%   buildHPP()                    rebuild the top model HPP.slx for the
%                                 asset counts N in HPP_parameters()
%   buildHPP(All=true)            also create controller and asset models
%                                 that do not exist yet (placeholder
%                                 templates); existing ones are kept
%   buildHPP(All=true, Force=true)  regenerate every controller and asset
%                                 model from the template: OVERWRITES them,
%                                 including any hand-made changes
%
%   Ownership: HPP.slx is always generated (its structure depends on N), so
%   never edit it by hand. Controller and asset models are normal,
%   hand-maintained models once created: develop them in the Simulink
%   editor; keep their port names and order (HPP.slx connects by port).
%
%   Structure (see docs/design.md):
%     models/HPP.slx                 top model: HPPC, plant controllers,
%                                    N asset instances per type, POC, delays
%     models/ctrl/<name>.slx         7 controllers, each a future DLL:
%                                    HPPC, WPPC, SPPC, BESSC, WT_ctrl,
%                                    PV_ctrl, BESS_ctrl
%     models/assets/<type>_asset.slx asset plant models; each contains one
%                                    asset controller instance
%
%   Each controller: inports -> one MATLAB Function block "logic" ->
%   outports, all double, per unit on its own base; tunable parameters are
%   the model argument "par"; sample time "Ts". Defaults for both come from
%   HPP_parameters() via the model workspace.
%
%   Changing N (p.wt.N, p.pv.N, p.bess.N) requires rerunning buildHPP.

    arguments
        opts.All (1,1) logical = false
        opts.Force (1,1) logical = false
    end

    root = currentProject().RootFolder;
    dirs.ctrl   = fullfile(root, "models", "ctrl");
    dirs.assets = fullfile(root, "models", "assets");
    dirs.top    = fullfile(root, "models");
    for d = [dirs.ctrl, dirs.assets]
        if ~isfolder(d), mkdir(d); end
    end

    p = HPP_parameters();
    % rebuilding a model that exists on the path warns about "shadowing"
    ws0 = warning("off", "Simulink:Engine:MdlFileShadowing");
    restoreWarn = onCleanup(@() warning(ws0));

    if opts.All
        % controllers before assets (assets reference the asset controllers)
        specs = controllerSpecs();
        for k = 1:numel(specs)
            if opts.Force || ~isfile(fullfile(dirs.ctrl, specs(k).name + ".slx"))
                if specs(k).name == "WT_ctrl"
                    buildWTCtrlType4A(dirs.ctrl);      % stage 3: IEC 61400-27-1 Type 4A
                elseif any(specs(k).name == ["WPPC", "SPPC", "BESSC"])
                    buildPlantCtrlRepc(specs(k).name, dirs.ctrl);      % WECC REPC_A plant controllers
                elseif any(specs(k).name == ["PV_ctrl", "BESS_ctrl"])
                    buildReecCtrl(extractBefore(specs(k).name, "_"), dirs.ctrl);   % stage 4: WECC REEC_B / REEC_C
                else
                    buildController(specs(k), dirs.ctrl);
                end
            else
                fprintf("Kept existing %s.slx\n", specs(k).name);
            end
        end
        for type = ["WT", "PV", "BESS"]
            if opts.Force || ~isfile(fullfile(dirs.assets, type + "_asset.slx"))
                buildAsset(type, dirs.assets);
            else
                fprintf("Kept existing %s_asset.slx\n", type);
            end
        end
    end

    buildTop(p, dirs.top);
end

%% ===================================================================
%  Controller definitions (stage 1: placeholder logic)
%  ===================================================================
function specs = controllerSpecs()
    specs = struct("name", {}, "cfield", {}, "inputs", {}, "outputs", {}, "code", {}, "doc", {});

    % ---- HPP controller (base: POC) ----
    % inputs grouped per source: TSO, POC, wind plant, PV plant, BESS plant
    in = ["P_ref","Q_ref","f_meas","P_poc","Q_poc","V_poc", ...
          "P_avail_wpp","P_wpp","Q_wpp", ...
          "P_avail_spp","P_spp","Q_spp", ...
          "P_chg_max_bess","P_dis_max_bess","SOC_bess","P_bess","Q_bess"];
    out = ["P_set_wpp","Q_set_wpp","P_set_spp","Q_set_spp","P_set_bess","Q_set_bess"];
    body = [
        "% Placeholder (stages 1-2): wind and PV deliver what is available, the BESS"
        "% covers the rest of P_ref within its limits (open loop: P_poc is not used,"
        "% so network losses are not compensated). Q_ref is split over the plants by"
        "% rating, also open loop."
        "P_set_wpp  = P_avail_wpp;"
        "P_set_spp  = P_avail_spp;"
        "P_set_bess = min(max(P_ref - P_avail_wpp - P_avail_spp, -P_chg_max_bess), P_dis_max_bess);"
        "Q_set_wpp  = Q_ref * par.qshare(1);"
        "Q_set_spp  = Q_ref * par.qshare(2);"
        "Q_set_bess = Q_ref * par.qshare(3);"];
    specs(end+1) = spec("HPPC", "hppc", dims(in), dims(out), body, ...
        "HPP controller. Per unit on p.ctrl.hppc.Pbase (POC base). Placeholder dispatch.");

    % ---- Plant controllers (base: plant rating; vectors of width N) ----
    for pc = [struct("name","WPPC","cfield","wppc","bess",false), ...
              struct("name","SPPC","cfield","sppc","bess",false), ...
              struct("name","BESSC","cfield","bessc","bess",true)]
        in  = dims(["P_set","Q_set","P_meas","Q_meas","V_meas"]);
        in(end+1) = struct("name", "P_avail_k", "dims", "N"); %#ok<AGROW>
        out = dims(["P_set_k","Q_set_k","P_avail","P_plant","Q_plant"]);
        body = [
            "% Stage 1 placeholder: split the plant set-point over the assets in"
            "% proportion to their availability (equal shares if nothing is available)."
            "s = sum(P_avail_k);"
            "if s > 1e-6"
            "    w = P_avail_k / s;"
            "else"
            "    w = par.share;"
            "end"
            "P_set_k = P_set * w;"
            "Q_set_k = Q_set * w;"
            "P_avail = s;"
            "P_plant = P_meas;"
            "Q_plant = Q_meas;"];
        if pc.bess
            in(end+1) = struct("name", "SOC_k", "dims", "N"); %#ok<AGROW>
        end
        in(end+1) = struct("name", "f_meas", "dims", "1"); %#ok<AGROW>
        if pc.bess
            out = [out, dims(["P_chg_max","P_dis_max","SOC"])]; %#ok<AGROW>
            body = [body; "P_chg_max = s;  % stage 1: symmetric limits, no SOC limits yet"; ...
                          "P_dis_max = s;"; "SOC = sum(par.share .* SOC_k);"]; %#ok<AGROW>
        end
        specs(end+1) = spec(pc.name, pc.cfield, in, out, body, ...
            pc.name + " plant controller. Per unit on the plant rating; asset vectors of width N."); %#ok<AGROW>
    end

    % ---- Asset controllers (base: asset rating) ----
    common = ["P_set","Q_set","V_term","P_meas","Q_meas"];
    assets = [struct("name","WT_ctrl",  "cfield","wt",   "src","P_wind_avail"), ...
              struct("name","PV_ctrl",  "cfield","pv",   "src","P_pv_avail"), ...
              struct("name","BESS_ctrl","cfield","bess", "src","SOC")];
    for a = assets
        in = dims([common, a.src]);
        out = dims(["ip_cmd","iq_cmd","P_avail"]);
        if a.cfield == "bess"
            body = [
                "% Stage 1 placeholder: follow the set-point within the power limit."
                "P_avail = par.Pmax;"
                "P = min(max(P_set, -par.Pmax), par.Pmax);"];
        else
            body = [
                "% Stage 1 placeholder: follow the set-point, limited to what is available."
                "P_avail = " + a.src + ";"
                "P = min(max(P_set, 0), P_avail);"];
        end
        body = [body; "v = max(V_term, par.Vmin);"; "ip_cmd = P / v;"; "iq_cmd = Q_set / v;"]; %#ok<AGROW>
        specs(end+1) = spec(a.name, a.cfield, in, out, body, ...
            a.name + " asset controller. Per unit on the asset rating."); %#ok<AGROW>
    end
end

function s = spec(name, cfield, in, out, body, doc)
    s = struct("name", name, "cfield", cfield, "inputs", in, "outputs", out, ...
               "code", body, "doc", doc);
end

function d = dims(names)
    d = struct("name", cellstr(names), "dims", "1");
end

%% ===================================================================
%  Top model builder
%
%  Top level (few lines, buses between the parts):
%    TSO ──▶ HPP controller ──set-points──▶ Wind / PV / BESS plant ──I──▶ Network
%                 ▲  ◀──────── plant measurements ─────┘   ▲                │
%                 └──────────── POC measurements ──────────┼────────────────┤
%                                                          └─ V, busbar ────┘
%  Buses carry SI units (W, var; voltages in pu). Each subsystem converts
%  to its controller's per-unit base and applies the communication delay
%  of the links it receives. The controller models keep flat scalar ports
%  (DLL interface); buses exist only in the wiring around them.
%  ===================================================================
function buildTop(p, folder)
    m = "HPP";
    closeIfLoaded(m);
    new_system(m);
    set_param(m, "SolverType", "Fixed-step", "Solver", "ode4", "FixedStep", "p.sim.Ts", ...
        "StopTime", "p.sim.Tstop", "SaveOutput", "on", "SignalLogging", "on", ...
        "AlgebraicLoopMsg", "error", ...
        "Description", "Hierarchical HPP model, generated by utilities/buildHPP.m. Do not edit by hand: rebuild instead.");

    plants = [ ...
        struct("title","Wind plant", "tag","wpp",  "ctrl","WPPC",  "c","wppc",  "type","WT",   "t","wt"), ...
        struct("title","PV plant",   "tag","spp",  "ctrl","SPPC",  "c","sppc",  "type","PV",   "t","pv"), ...
        struct("title","BESS plant", "tag","bess", "ctrl","BESSC", "c","bessc", "type","BESS", "t","bess")];

    buildTSO(m + "/TSO");
    buildHppcSub(m + "/HPP controller");
    offset = 0;
    for i = 1:3
        buildPlantSub(m + "/" + plants(i).title, p, plants(i), i, offset);
        offset = offset + p.(plants(i).t).N;
    end
    buildNetwork(m + "/Network");
    buildLogging(m + "/Logging");

    % ---- fixed top-level layout (left to right), before drawing lines ----
    %   TSO | HPP controller | Wind / PV / BESS plant | Network | Logging | outports
    % Blocks are aligned to each other's ports so the forward lines are
    % straight: HPPC output i level with plant i's set-point input, the
    % plants' current outputs level with the Network inputs, TSO level with
    % HPPC input 1. (Gaps leave room for the Goto/From blocks.)
    D = 150;                                           % plant pitch
    setPos(m + "/HPP controller", [230 60 400 60 + 3*D]);   % 3 outputs, D apart
    hP = 96;
    for i = 1:3
        yo = portY(m + "/HPP controller", "out", i);
        setPos(m + "/" + plants(i).title, [610, yo - hP/4, 790, yo + 3*hP/4]);   % input 1 at hP/4
    end
    yI = portY(m + "/Wind plant", "out", 2);           % current output of the first plant
    setPos(m + "/Network", [990, yI - D/2, 1190, yI - D/2 + 3*D]);
    yT = portY(m + "/HPP controller", "in", 1);
    setPos(m + "/TSO", [20, yT - 35, 120, yT + 35]);
    setPos(m + "/Logging", [1510 60 1630 60 + 12*36]);

    % block colours by role: controllers blue, plants green, network grey
    set_param(m + "/HPP controller", "BackgroundColor", areaColor("controller"));
    for i = 1:3, set_param(m + "/" + plants(i).title, "BackgroundColor", areaColor("asset")); end
    set_param(m + "/Network", "BackgroundColor", areaColor("interface"));

    % ---- top-level wiring ----
    % Forward chain as lines: TSO -> HPPC -> plants -> network.
    % Feedback and logging signals as colour-coded Goto/From pairs
    % (DTU palette): orange = plant measurements to the HPPC, green =
    % network to plants and HPPC, grey = to logging only.
    orange = "[0.988 0.463 0.204]";  green = "[0.122 0.816 0.510]";  grey = "[0.855 0.855 0.855]";
    ln(m, "TSO/1", "HPP controller/1", "TSO");
    for i = 1:3
        pt = plants(i).title;
        ln(m, sprintf("HPP controller/%d", i), pt + "/1", "set-points " + plants(i).tag);
        ln(m, pt + "/2", sprintf("Network/%d", i), "I " + plants(i).tag);
    end
    gotoFrom(m, "Wind plant/1", "HPP controller/3", "meas_wpp", orange, "meas wpp");
    gotoFrom(m, "PV plant/1", "HPP controller/4", "meas_spp", orange, "meas spp");
    gotoFrom(m, "BESS plant/1", ["HPP controller/5", "Logging/4"], "meas_bess", orange, "meas bess");
    gotoFrom(m, "Network/1", ["Wind plant/2", "PV plant/2", "BESS plant/2", "Logging/1"], "network", green, "network");
    gotoFrom(m, "Network/2", ["HPP controller/2", "Logging/2"], "POC_meas", green, "POC meas");
    % BESS set-point for logging: short branch off the set-point bus to a
    % Goto just above the line (below it is the plant's network From)
    yb = portY(m + "/HPP controller", "out", 3);
    add_block("simulink/Signal Routing/Goto", m + "/set-points_bess goto", "GotoTag", "setpoints_bess", ...
        "TagVisibility", "local", "BackgroundColor", grey, "ShowName", "off", ...
        "Position", [450, yb - 48, 570, yb - 26]);
    ln(m, "HPP controller/3", "set-points_bess goto/1", "");
    lh = get_param(m + "/Logging", "PortHandles");  lpos = get_param(lh.Inport(3), "Position");
    add_block("simulink/Signal Routing/From", m + "/set-points_bess from", "GotoTag", "setpoints_bess", ...
        "BackgroundColor", grey, "ShowName", "off", "Position", [lpos(1)-160, lpos(2)-11, lpos(1)-40, lpos(2)+11]);
    ln(m, "set-points_bess from/1", "Logging/3", "");

    logNames = ["P_poc","SOC","P_bess_set","Q_poc","P_wpp","P_spp","P_bess","Q_wpp","Q_spp","Q_bess","V_poc","V_bus"];
    lp = get_param(m + "/Logging", "PortConnectivity");
    for k = 1:numel(logNames)
        y = lp(4 + k).Position(2);        % align each outport with its Logging port
        add_block("simulink/Sinks/Out1", m + "/" + logNames(k), "Port", num2str(k), ...
            "Position", [1710 y-7 1740 y+7]);
        ln(m, sprintf("Logging/%d", k), logNames(k) + "/1", logNames(k));
    end

    % record what this build was made for; runCase/runSweep check it
    ws = get_param(m, "ModelWorkspace");
    assignin(ws, "buildInfo", struct("N_wt", p.wt.N, "N_pv", p.pv.N, "N_bess", p.bess.N, ...
        "built", string(datetime("now"))));
    save_system(m, fullfile(folder, "HPP.slx"));
    fprintf("Built HPP.slx with N = %d wind, %d PV, %d BESS asset instances.\n", p.wt.N, p.pv.N, p.bess.N);
end

% ---------------------------------------------------------------------
function buildTSO(sys)
    % TSO / market set-points and grid frequency (constants for now),
    % each constant level with its Bus Creator input
    newSubsystem(sys);
    names = ["P_ref", "Q_ref", "f_meas"];
    vals  = ["p.grid.Pref", "0", "1"];
    add_block("simulink/Signal Routing/Bus Creator", sys + "/TSO bus", "Inputs", "3", ...
        "Position", [260 40 265 160]);
    for j = 1:3
        if names(j) == "f_meas"   % grid frequency with the optional step (p.grid.fev)
            add_block("simulink/Sources/Step", sys + "/" + names(j), "Time", "p.grid.fev.t0", ...
                "Before", "1", "After", "1 + p.grid.fev.df", "SampleTime", "0", "Position", [0 0 90 26]);
        else
            add_block("simulink/Sources/Constant", sys + "/" + names(j), "Value", vals(j), "Position", [0 0 90 26]);
        end
        rowInto(sys, "TSO bus/" + j, names(j), names(j), 50);
    end
    add_block("simulink/Sinks/Out1", sys + "/TSO", "Port", "1", "Position", [0 0 30 14]);
    rowFrom(sys, "TSO bus/1", "TSO", "", 50);
    set_param(sys, "Description", "TSO / market set-points: P_ref [W], Q_ref [var], f_meas [pu].");
    hideStructuralNames(sys);
end

% ---------------------------------------------------------------------
function buildNetwork(sys)
    % Masked subsystem: positive-sequence phasor network (collector
    % system, transformers, grid), solved as V = Z*I every step in one
    % MATLAB Function block. Mask parameter "net" = p.net.solver
    % (networkSolverData). In: current injections per plant (buses,
    % system pu). Out: 1 network bus to the plants (V_re, V_im of all
    % asset terminals one step old, P_plant / Q_plant [W / var] per plant
    % into the busbar, V_bus [pu]); 2 POC bus (P_poc [W], Q_poc [var],
    % V_poc [pu]), power into the grid.
    newSubsystem(sys);
    tags = ["wpp", "spp", "bess"];
    for i = 1:3
        inport(sys, "I " + tags(i), i);
    end
    add_block("simulink/Signal Routing/Mux", sys + "/I_re all", "Inputs", "3");
    add_block("simulink/Signal Routing/Mux", sys + "/I_im all", "Inputs", "3");
    for i = 1:3
        sel = sprintf("I sel %d", i);
        busSelect(sys, sel, "I " + tags(i) + "/1", ["I_re", "I_im"]);
        ln(sys, sel + "/1", sprintf("I_re all/%d", i), "");
        ln(sys, sel + "/2", sprintf("I_im all/%d", i), "");
    end

    outs = ["V_re", "V_im", "P_plant", "Q_plant", "V_bus", "P_poc", "Q_poc", "V_poc"];
    body = [
        "Z  = complex(net.Zre, net.Zim);"
        "nA = numel(I_re);"
        "n  = size(Z, 1);"
        "Zg = complex(net.Zg_re, net.Zg_im);"
        "I = complex(zeros(n, 1), zeros(n, 1));"
        "I(1:nA) = complex(I_re(:), I_im(:));"
        "I(n) = I(n) + E / Zg;                     % grid Norton source at the POC (E: grid voltage, incl. dips)"
        "V = Z * I;"
        "V_re = real(V(1:nA));"
        "V_im = imag(V(1:nA));"
        "% power each feeder delivers into the busbar (incl. its bus-end charging)"
        "Vb = V(n-1);"
        "Vc = V(nA+1:2*nA);"
        "Ifeed = (Vc - Vb) ./ complex(net.Zcab_re, net.Zcab_im) - 1i * net.Bhalf .* Vb;"
        "Sfeed = Vb .* conj(Ifeed);"
        "P_plant = zeros(3, 1);  Q_plant = zeros(3, 1);"
        "for k = 1:nA"
        "    P_plant(net.plantOf(k)) = P_plant(net.plantOf(k)) + real(Sfeed(k));"
        "    Q_plant(net.plantOf(k)) = Q_plant(net.plantOf(k)) + imag(Sfeed(k));"
        "end"
        "V_bus = abs(Vb);"
        "Vp = V(n);"
        "Spoc = Vp * conj((Vp - E) / Zg);          % power into the grid"
        "P_poc = real(Spoc);  Q_poc = imag(Spoc);  V_poc = abs(Vp);"];
    addMatlabFunction(sys + "/solve", "solve", ["I_re", "I_im", "E"], outs, body, "net", [0 0 180 320]);
    ln(sys, "I_re all/1", "solve/1", "I_re");
    ln(sys, "I_im all/1", "solve/2", "I_im");
    % grid source voltage with an optional dip (p.grid.dip: t0, dur, depth)
    add_block("simulink/Sources/Constant", sys + "/E0", "Value", "net.E");
    add_block("simulink/Sources/Step", sys + "/dip start", "Time", "p.grid.dip.t0", ...
        "Before", "0", "After", "p.grid.dip.depth", "SampleTime", "0");
    add_block("simulink/Sources/Step", sys + "/dip end", "Time", "p.grid.dip.t0 + p.grid.dip.dur", ...
        "Before", "0", "After", "p.grid.dip.depth", "SampleTime", "0");
    add_block("simulink/Math Operations/Sum", sys + "/E grid", "Inputs", "+-+");
    ln(sys, "E0/1", "E grid/1", "");  ln(sys, "dip start/1", "E grid/2", "");  ln(sys, "dip end/1", "E grid/3", "");
    ln(sys, "E grid/1", "solve/3", "E");

    % terminal voltages one step old (generator "PLL"; avoids an algebraic loop)
    add_block("simulink/Discrete/Memory", sys + "/V_re prev", "InitialCondition", "ones(net.nA,1)");
    add_block("simulink/Discrete/Memory", sys + "/V_im prev", "InitialCondition", "zeros(net.nA,1)");
    ln(sys, "solve/1", "V_re prev/1", "");
    ln(sys, "solve/2", "V_im prev/1", "");
    % buses out; powers from system pu to W / var
    busCreator(sys, "network bus", 6);
    ln(sys, "V_re prev/1", "network bus/1", "V_re");
    ln(sys, "V_im prev/1", "network bus/2", "V_im");
    chain(sys, "solve/3", "gain:net.Sb", "network bus/3", "P_plant to W", "P_plant");
    chain(sys, "solve/4", "gain:net.Sb", "network bus/4", "Q_plant to var", "Q_plant");
    ln(sys, "solve/5", "network bus/5", "V_bus");
    % grid frequency [pu] with the optional step (p.grid.fev), same at all nodes
    add_block("simulink/Sources/Step", sys + "/f grid", "Time", "p.grid.fev.t0", ...
        "Before", "1", "After", "1 + p.grid.fev.df", "SampleTime", "0");
    ln(sys, "f grid/1", "network bus/6", "f");
    busCreator(sys, "POC bus", 3);
    chain(sys, "solve/6", "gain:net.Sb", "POC bus/1", "P_poc to W", "P_poc");
    chain(sys, "solve/7", "gain:net.Sb", "POC bus/2", "Q_poc to var", "Q_poc");
    ln(sys, "solve/8", "POC bus/3", "V_poc");
    outport(sys, "network", "network bus/1", 1);
    outport(sys, "POC meas", "POC bus/1", 2);

    mk = Simulink.Mask.create(sys);
    mk.addParameter("Name", "net", "Prompt", "Network solver data", "Value", "p.net.solver", ...
        "Evaluate", "on", "Tunable", "off");
    mk.Description = "Positive-sequence phasor network (collector system, transformers, grid). " + ...
        "Solves V = Z*I each step. Data from networkSolverData(p).";
    hideStructuralNames(sys);
    Simulink.BlockDiagram.arrangeSystem(sys);
end

% ---------------------------------------------------------------------
function buildLogging(sys)
    % Picks the logged signals out of the buses; its 12 outputs feed the
    % top-level outports (out.yout). One straight row per logged signal.
    newSubsystem(sys);
    order = ["P_poc","SOC","P_bess_set","Q_poc","P_wpp","P_spp","P_bess","Q_wpp","Q_spp","Q_bess","V_poc","V_bus"];
    port = @(n) num2str(find(order == n));
    pitch = 34;  xs = 200;  y = 40;

    % POC: P_poc, Q_poc, V_poc
    inport(sys, "POC meas", 2);
    add_block("simulink/Signal Routing/Bus Selector", sys + "/POC sel", "Position", [xs y xs+5 y+3*pitch]);
    rowInto(sys, "POC sel/1", "POC meas", "", 50);
    set_param(sys + "/POC sel", "OutputSignals", "P_poc,Q_poc,V_poc");
    for e = ["P_poc", "Q_poc", "V_poc"]
        add_block("simulink/Sinks/Out1", sys + "/" + e, "Port", port(e), "Position", [0 0 30 14]);
        rowFrom(sys, "POC sel/" + find(["P_poc", "Q_poc", "V_poc"] == e), e, "", 200);
    end
    y = y + 3*pitch + 20;

    % P_bess_set (from the HPPC's set-point bus to the BESS plant) and SOC
    inport(sys, "set-points bess", 3);
    add_block("simulink/Signal Routing/Bus Selector", sys + "/set-points sel", "Position", [xs y xs+5 y+pitch]);
    rowInto(sys, "set-points sel/1", "set-points bess", "", 50);
    set_param(sys + "/set-points sel", "OutputSignals", "P_set");
    add_block("simulink/Sinks/Out1", sys + "/P_bess_set", "Port", port("P_bess_set"), "Position", [0 0 30 14]);
    rowFrom(sys, "set-points sel/1", "P_bess_set", "", 200);
    y = y + pitch + 20;
    inport(sys, "meas bess", 4);
    add_block("simulink/Signal Routing/Bus Selector", sys + "/SOC sel", "Position", [xs y xs+5 y+pitch]);
    rowInto(sys, "SOC sel/1", "meas bess", "", 50);
    set_param(sys + "/SOC sel", "OutputSignals", "SOC_plant");
    add_block("simulink/Sinks/Out1", sys + "/SOC", "Port", port("SOC"), "Position", [0 0 30 14]);
    rowFrom(sys, "SOC sel/1", "SOC", "", 200);
    y = y + pitch + 20;

    % network: P per plant, Q per plant, busbar voltage
    inport(sys, "network", 1);
    add_block("simulink/Signal Routing/Bus Selector", sys + "/network sel", ...
        "Position", [xs y xs+5 y+3*(3*pitch+30)]);
    rowInto(sys, "network sel/1", "network", "", 50);
    set_param(sys + "/network sel", "OutputSignals", "P_plant,Q_plant,V_bus");
    % selector outputs are evenly spaced; place each group's demux on its output
    outs = {["P_wpp", "P_spp", "P_bess"], ["Q_wpp", "Q_spp", "Q_bess"]};
    for j = 1:2
        dm = ["P plants", "Q plants"]; dm = dm(j);
        add_block("simulink/Signal Routing/Demux", sys + "/" + dm, "Outputs", "3", "Position", [0 0 5 3*pitch]);
        rowFrom(sys, "network sel/" + j, dm, "", 60);
        for k = 1:3
            n = outs{j}(k);
            add_block("simulink/Sinks/Out1", sys + "/" + n, "Port", port(n), "Position", [0 0 30 14]);
            rowFrom(sys, dm + "/" + k, n, "", 135);
        end
    end
    add_block("simulink/Sinks/Out1", sys + "/V_bus", "Port", port("V_bus"), "Position", [0 0 30 14]);
    rowFrom(sys, "network sel/3", "V_bus", "", 200);
    % ports were created in row order; set the final numbers explicitly
    renumberPorts(sys, order);
    renumberPorts(sys, ["network", "POC meas", "set-points bess", "meas bess"]);
    set_param(sys, "Description", "Selects the logged signals (W, var, pu) for the top-level outports.");
    hideStructuralNames(sys);
end

%% ===================================================================
%  Small helpers for building systems
%  ===================================================================
function newSubsystem(sys)
    % empty subsystem block
    add_block("simulink/Ports & Subsystems/Subsystem", sys);
    delete_line(sys, "In1/1", "Out1/1");
    delete_block(sys + "/In1");
    delete_block(sys + "/Out1");
end

function inport(sys, name, k)
    add_block("simulink/Sources/In1", sys + "/" + name, "Port", num2str(k));
end

function outport(sys, name, src, k)
    add_block("simulink/Sinks/Out1", sys + "/" + name, "Port", num2str(k));
    ln(sys, src, name + "/1", name);
end

function ln(sys, a, b, name)
    % line from port a to port b; name it if a name is given
    h = add_line(sys, a, b, "autorouting", "smart");
    srcType = get_param(sys + "/" + extractBefore(a, "/"), "BlockType");
    if name ~= "" && srcType ~= "BusSelector"   % bus selector outputs keep the element name
        set_param(h, "Name", name);
    end
end

function busSelect(sys, name, src, names)
    % bus selector with one output per name, in the given order
    add_block("simulink/Signal Routing/Bus Selector", sys + "/" + name);
    ln(sys, src, name + "/1", "");
    set_param(sys + "/" + name, "OutputSignals", char(strjoin(names, ",")));
end

function gotoFrom(sys, src, dsts, tag, color, lineName)
    % Goto next to the source port and a From next to each destination
    % port, all with the same tag and background colour
    w = 12 + 8 * strlength(tag);
    srcBlk = extractBefore(src, "/");  k = str2double(extractAfter(src, "/"));
    ph = get_param(sys + "/" + srcBlk, "PortHandles");
    pos = get_param(ph.Outport(k), "Position");
    g = tag + " goto";
    add_block("simulink/Signal Routing/Goto", sys + "/" + g, "GotoTag", tag, "TagVisibility", "local", ...
        "BackgroundColor", color, "ShowName", "off", "Position", [pos(1)+40, pos(2)-11, pos(1)+40+w, pos(2)+11]);
    ln(sys, src, g + "/1", lineName);
    for i = 1:numel(dsts)
        dBlk = extractBefore(dsts(i), "/");  j = str2double(extractAfter(dsts(i), "/"));
        dh = get_param(sys + "/" + dBlk, "PortHandles");
        dpos = get_param(dh.Inport(j), "Position");
        f = sprintf("%s from %d", tag, i);
        add_block("simulink/Signal Routing/From", sys + "/" + f, "GotoTag", tag, ...
            "BackgroundColor", color, "ShowName", "off", "Position", [dpos(1)-40-w, dpos(2)-11, dpos(1)-40, dpos(2)+11]);
        ln(sys, f + "/1", dsts(i), "");
    end
end

function setPos(blk, rect)
    set_param(blk, "Position", rect);
end

function busCreator(sys, name, n)
    % empty Bus Creator with n inputs; connect elements with chain(...,
    % outName) or ln(..., name) so each input line carries its element name
    add_block("simulink/Signal Routing/Bus Creator", sys + "/" + name, "Inputs", num2str(n));
end

function setArg(blk, argName, value)
    ip = get_param(blk, "InstanceParameters");
    k = strcmp({ip.Name}, argName);
    assert(any(k), "Model block %s has no argument %s", blk, argName);
    ip(k).Value = char(value);
    set_param(blk, "InstanceParameters", ip);
end

function addMatlabFunction(blk, fname, inNames, outNames, body, paramName, pos)
    % MATLAB Function block with the given ports; paramName ("" for none)
    % becomes a Parameter data item resolved from the model / mask workspace
    add_block("simulink/User-Defined Functions/MATLAB Function", blk, "Position", pos);
    rt = sfroot;
    ch = rt.find("-isa", "Stateflow.EMChart", "Path", char(blk));
    args = inNames;
    if paramName ~= "", args(end+1) = paramName; end
    sig = "function [" + strjoin(outNames, ", ") + "] = " + fname + "(" + strjoin(args, ", ") + ")";
    ch.Script = char(strjoin([sig; "%#codegen"; body(:); "end"], newline));
    if paramName ~= ""
        d = ch.find("-isa", "Stateflow.Data", "Name", char(paramName));
        d.Scope = 'Parameter';
    end
end

function registerInProject(file)
    % add a newly generated model to the MATLAB project (no-op if already in it)
    pr = currentProject;
    if isempty(findFile(pr, file))
        addFile(pr, file);
    end
end

function closeIfLoaded(mdl)
    if bdIsLoaded(mdl)
        close_system(mdl, 0);
    end
end

%% ===================================================================
%  Controller model builder (template; created only if missing unless
%  Force=true). Ports are placed level with the logic block's ports, so
%  every line is straight.
%  ===================================================================
function buildController(s, folder)
    mdl = string(s.name);
    closeIfLoaded(mdl);
    new_system(mdl);
    set_param(mdl, "SolverType", "Fixed-step", "Solver", "FixedStepDiscrete", ...
        "FixedStep", "Ts", "StopTime", "inf", "Description", s.doc);

    usesN = any(string({s.inputs.dims}) == "N");
    ws = get_param(mdl, "ModelWorkspace");
    ws.DataSource = "MATLAB Code";
    code = [
        "% Defaults for standalone use and code generation, from HPP_parameters."
        "% In HPP.slx the Model block passes p.ctrl." + s.cfield + ".par as the argument ""par""."
        "pDefault = HPP_parameters();"
        "Ts  = pDefault.ctrl." + s.cfield + ".Ts;    % [s] fixed sample time"
        "par = pDefault.ctrl." + s.cfield + ".par;   % tunable parameters (model argument)"];
    if usesN
        code(end+1) = "N   = numel(par.share);   % asset instances (fixed per build)";
    end
    code(end+1) = "clear pDefault";
    ws.MATLABCode = char(strjoin(code, newline));
    reload(ws);
    set_param(mdl, "ParameterArgumentNames", "par");

    inNames  = string({s.inputs.name});
    inDims   = string({s.inputs.dims});
    outNames = string({s.outputs.name});
    nIn = numel(inNames); nOut = numel(outNames);
    h = 60 * max(nIn, nOut);
    addMatlabFunction(mdl + "/logic", "logic", inNames, outNames, s.code, "par", [400 40 640 40+h]);
    for k = 1:nIn
        add_block("simulink/Sources/In1", mdl + "/" + inNames(k), "Port", num2str(k), ...
            "OutDataTypeStr", "double", "PortDimensions", inDims(k), "SampleTime", "Ts", ...
            "Position", [0 0 30 14]);
        rowInto(mdl, "logic/" + k, inNames(k), "", 120);
    end
    for k = 1:nOut
        add_block("simulink/Sinks/Out1", mdl + "/" + outNames(k), "Port", num2str(k), ...
            "OutDataTypeStr", "double", "Position", [0 0 30 14]);
        rowFrom(mdl, "logic/" + k, outNames(k), "", 120);
    end
    save_system(mdl, fullfile(folder, mdl + ".slx"));
    close_system(mdl);
    registerInProject(fullfile(folder, mdl + ".slx"));
end

%% ===================================================================
%  Asset model builder (plant side, not a DLL; template, created only if
%  missing unless Force=true)
%    inputs (left) -> asset controller -> generator -> outputs (right)
%    terminal voltage and measured P, Q via local Goto/From
%  ===================================================================
function buildAsset(type, folder)
    mdl = string(type + "_asset");
    ctrl = type + "_ctrl";
    t = lower(type);
    closeIfLoaded(mdl);
    new_system(mdl);
    set_param(mdl, "SolverType", "Fixed-step", "Solver", "ode4", "FixedStep", "Tsim", "StopTime", "inf", ...
        "Description", type + " asset: " + ctrl + " (asset controller) + generator / converter model " + ...
                       "(WT: IEC 61400-27-1 Type 4 generator system; PV, BESS: WECC REGC_A) feeding a " + ...
                       "current source aligned with the terminal voltage. Per unit on the asset rating " + ...
                       "(voltage on the nominal voltage). Argument ap = p." + t + ".ap(k).");
    ws = get_param(mdl, "ModelWorkspace");
    ws.DataSource = "MATLAB Code";
    ws.MATLABCode = char(strjoin([
        "% Defaults for standalone use. In HPP.slx each instance k gets p." + t + ".ap(k)."
        "pDefault = HPP_parameters();"
        "ap   = pDefault." + t + ".ap(1);   % asset arguments (incl. controller par)"
        "Tsim = pDefault.sim.Ts;"
        "clear pDefault"], newline));
    reload(ws);
    set_param(mdl, "ParameterArgumentNames", "ap");
    green = "[0.122 0.816 0.510]";  orange = "[0.988 0.463 0.204]";

    % asset controller, ports 1 P_set 2 Q_set 3 V_term 4 P_meas 5 Q_meas 6 source
    cb = mdl + "/ctrl";
    add_block("simulink/Ports & Subsystems/Model", cb, "ModelName", ctrl, "Position", [520 60 700 420]);
    Simulink.ModelReference.refresh(cb);
    setArg(cb, "par", "ap.ctrl");
    set_param(cb, "BackgroundColor", areaColor("controller"));

    % inports: set-points and source straight into the controller
    add_block("simulink/Sources/In1", mdl + "/P_set", "Port", "1", "OutDataTypeStr", "double", "Position", [0 0 30 14]);
    add_block("simulink/Sources/In1", mdl + "/Q_set", "Port", "2", "OutDataTypeStr", "double", "Position", [0 0 30 14]);
    rowInto(mdl, "ctrl/1", "P_set", "P_set", 300);
    rowInto(mdl, "ctrl/2", "Q_set", "Q_set", 300);
    % terminal voltage phasor -> local Goto (used by the controller and the generator)
    for j = 1:2
        v = ["V_re", "V_im"]; v = v(j);
        add_block("simulink/Sources/In1", mdl + "/" + v, "Port", num2str(2 + j), "OutDataTypeStr", "double", ...
            "Position", [40, 460 + 50*j, 70, 474 + 50*j]);
        g = addGoto(mdl, v, green);
        rowFrom(mdl, v + "/1", g, v, 40);
    end
    % |V| for the controller
    addMatlabFunction(mdl + "/terminal", "terminal", ["V_re", "V_im"], "V_term", ...
        "V_term = sqrt(V_re^2 + V_im^2);", "", [0 0 90 60]);
    set_param(mdl + "/terminal", "ShowName", "off");   % the block shows "terminal" itself
    rowInto(mdl, "ctrl/3", "terminal", "V_term", 60);
    rowInto(mdl, "terminal/1", addFrom(mdl, "V_re", green), "", 30);
    rowInto(mdl, "terminal/2", addFrom(mdl, "V_im", green), "", 30);
    % measured P, Q (one step old) back into the controller
    for j = 1:2
        s = ["P", "Q"]; s = s(j);
        add_block("simulink/Discrete/Memory", mdl + "/" + s + " meas", "Position", [0 0 30 30]);
        rowInto(mdl, "ctrl/" + (3 + j), [addFrom(mdl, s, orange), s + " meas"], s + "_meas", 40);
    end
    % primary source
    switch type
        case "WT", src = "P_wind_avail";
        case "PV", src = "P_pv_avail";
        otherwise, src = "";
    end
    if src ~= ""
        add_block("simulink/Sources/In1", mdl + "/" + src, "Port", "5", "OutDataTypeStr", "double", "Position", [0 0 30 14]);
        rowInto(mdl, "ctrl/6", src, src, 300);
    else
        rowInto(mdl, "ctrl/6", addFrom(mdl, "SOC", orange), "SOC", 300);
    end

    % generator placeholder (WECC REGC-like, no dynamics yet): current source
    % aligned with the terminal voltage, I = (ip - j*iq) * V/|V|,
    % so P = |V|*ip and Q = |V|*iq (+ = export / capacitive)
    gen = [
        "v  = max(sqrt(V_re^2 + V_im^2), 1e-3);"
        "cr = V_re / v;  ci = V_im / v;"
        "I_re = ip_cmd*cr + iq_cmd*ci;"
        "I_im = ip_cmd*ci - iq_cmd*cr;"
        "P = v * ip_cmd;"
        "Q = v * iq_cmd;"];
    % same port spacing as the controller (6 ports over its height), first
    % two ports level with the controller's ip_cmd / iq_cmd
    cpos = get_param(cb, "Position");
    cy1 = portY(cb, "out", 1);  cy2 = portY(cb, "out", 2);  sp = cy2 - cy1;
    addMatlabFunction(mdl + "/generator", "generator", ["ip_cmd", "iq_cmd", "V_re", "V_im"], ...
        ["P", "Q", "I_re", "I_im"], gen, "", [cpos(3)+320, cy1 - sp/2, cpos(3)+460, cy1 - sp/2 + 4*sp]);
    if type == "WT"
        % IEC 61400-27-1 Type 4 generator system: current rate limits, then
        % a first-order lag Tg (active current: only its rise is limited)
        add_block("simulink/Discontinuities/Rate Limiter", mdl + "/dipmax", "Position", [0 0 40 30], ...
            "RisingSlewLimit", "ap.gen.dipmax", "FallingSlewLimit", "-1e6");
        add_block("simulink/Continuous/Transfer Fcn", mdl + "/Tg ip", "Position", [0 0 70 34], ...
            "Numerator", "1", "Denominator", "[ap.gen.Tg 1]");
        add_block("simulink/Discontinuities/Rate Limiter", mdl + "/diqmax", "Position", [0 0 40 30], ...
            "RisingSlewLimit", "ap.gen.diqmax", "FallingSlewLimit", "-ap.gen.diqmax");
        add_block("simulink/Continuous/Transfer Fcn", mdl + "/Tg iq", "Position", [0 0 70 34], ...
            "Numerator", "1", "Denominator", "[ap.gen.Tg 1]");
        rowFrom(mdl, "ctrl/1", ["dipmax", "Tg ip"], "ip_cmd", 40);
        rowFrom(mdl, "ctrl/2", ["diqmax", "Tg iq"], "iq_cmd", 40);
        ln(mdl, "Tg ip/1", "generator/1", "ip");
        ln(mdl, "Tg iq/1", "generator/2", "iq");
    else
        % WECC REGC_A: Ip recovery ramp Rrpwr, Iq ramps Iqrmax/Iqrmin, lag Tg;
        % low voltage active current management (LVACM) and low voltage
        % power logic (LVPL, static, on the unfiltered voltage)
        add_block("simulink/Discontinuities/Rate Limiter", mdl + "/Rrpwr", "Position", [0 0 40 30], ...
            "RisingSlewLimit", "ap.gen.Rrpwr", "FallingSlewLimit", "-1e6");
        add_block("simulink/Continuous/Transfer Fcn", mdl + "/Tg ip", "Position", [0 0 70 34], ...
            "Numerator", "1", "Denominator", "[ap.gen.Tg 1]");
        add_block("simulink/Discontinuities/Rate Limiter", mdl + "/Iqrmax", "Position", [0 0 40 30], ...
            "RisingSlewLimit", "ap.gen.Iqrmax", "FallingSlewLimit", "ap.gen.Iqrmin");
        add_block("simulink/Continuous/Transfer Fcn", mdl + "/Tg iq", "Position", [0 0 70 34], ...
            "Numerator", "1", "Denominator", "[ap.gen.Tg 1]");
        lv = [
            "v = sqrt(V_re^2 + V_im^2);"
            "g = ap.gen;"
            "% LVPL: limit on Ip, 0 at Zerox rising to Lvpl1 at Brkpt"
            "ipl = 1e6;"
            "if g.Lvplsw ~= 0"
            "    if v <= g.Zerox"
            "        ipl = 0;"
            "    elseif v < g.Brkpt"
            "        ipl = g.Lvpl1 * (v - g.Zerox) / (g.Brkpt - g.Zerox);"
            "    end"
            "end"
            "ip_l = min(max(ip, -ipl), ipl);"
            "% LVACM: Ip scaled to 0 between Lvpnt1 and Lvpnt0"
            "k = min(max((v - g.Lvpnt0) / (g.Lvpnt1 - g.Lvpnt0), 0), 1);"
            "ip_out = k * ip_l;"];
        addMatlabFunction(mdl + "/LVPL LVACM", "lvpl_lvacm", ["ip", "V_re", "V_im"], "ip_out", lv, "ap", [0 0 110 70]);
        rowFrom(mdl, "ctrl/1", ["Rrpwr", "Tg ip"], "ip_cmd", 30);
        rowFrom(mdl, "ctrl/2", ["Iqrmax", "Tg iq"], "iq_cmd", 30);
        tp = get_param(mdl + "/Tg ip", "Position");
        place(mdl + "/LVPL LVACM", tp(3) + 30, portY(mdl + "/Tg ip", "out", 1) + 20);
        ln(mdl, "Tg ip/1", "LVPL LVACM/1", "");
        ln(mdl, "LVPL LVACM/1", "generator/1", "ip");
        ln(mdl, "Tg iq/1", "generator/2", "iq");
        lp = get_param(mdl + "/LVPL LVACM", "Position");
        for j = 1:2
            vn = ["V_re", "V_im"]; f = addFrom(mdl, vn(j), green);
            place(mdl + "/" + f, lp(1) - 70, portY(mdl + "/LVPL LVACM", "in", j + 1));
            ln(mdl, f + "/1", "LVPL LVACM/" + (j + 1), "");
        end
    end
    rowInto(mdl, "generator/3", addFrom(mdl, "V_re", green), "", 30);
    rowInto(mdl, "generator/4", addFrom(mdl, "V_im", green), "", 30);
    % P_avail from the controller: Goto right next to it, out at the right
    rowFrom(mdl, "ctrl/3", addGoto(mdl, "P_avail", orange), "P_avail", 30);

    % outports: 1 P, 2 Q, 3 P_avail, 4 I_re, 5 I_im, 6 SOC (BESS)
    for j = 1:4
        names = ["P", "Q", "I_re", "I_im"];  ports = [1 2 4 5];
        add_block("simulink/Sinks/Out1", mdl + "/" + names(j), "Port", num2str(ports(j)), "Position", [0 0 30 14]);
        rowFrom(mdl, "generator/" + j, names(j), names(j), 160);
    end
    for s = ["P", "Q"]   % also to the measurement memories (branch to a Goto)
        gp = get_param(mdl + "/" + s, "Position");
        g = addGoto(mdl, s, orange);
        place(mdl + "/" + g, gp(1) - 110, gp(2) + 40);
        ln(mdl, "generator/" + find(["P", "Q"] == s), g + "/1", "");
    end
    add_block("simulink/Sinks/Out1", mdl + "/P_avail", "Port", "3", "Position", [0 0 30 14]);
    gp = get_param(mdl + "/I_im", "Position");
    f = addFrom(mdl, "P_avail", orange);
    place(mdl + "/" + f, gp(1) - 140, gp(2) + 80);
    rowFrom(mdl, f + "/1", "P_avail", "", 40);

    if type == "BESS"
        % battery energy: dSOC/dt = -kSOC * P  (P in pu, + = discharge)
        add_block("simulink/Math Operations/Gain", mdl + "/kSOC", "Gain", "-ap.kSOC", "Position", [0 0 60 30]);
        add_block("simulink/Continuous/Integrator", mdl + "/SOC int", "InitialCondition", "ap.SOC0", "Position", [0 0 30 30]);
        f = addFrom(mdl, "P", orange);
        place(mdl + "/" + f, gp(1) - 420, gp(2) + 150);
        rowFrom(mdl, f + "/1", ["kSOC", "SOC int"], "", 40);
        add_block("simulink/Sinks/Out1", mdl + "/SOC", "Port", "6", "Position", [0 0 30 14]);
        rowFrom(mdl, "SOC int/1", "SOC", "SOC", 140);
        g = addGoto(mdl, "SOC", orange);       % place first, then draw the branch
        sp2 = get_param(mdl + "/SOC", "Position");
        place(mdl + "/" + g, sp2(1) - 90, sp2(2) + 45);
        ln(mdl, "SOC int/1", g + "/1", "");
    end
    % outports were created out of order; set the final numbers explicitly
    outOrder = ["P", "Q", "P_avail", "I_re", "I_im"];
    if type == "BESS", outOrder(end+1) = "SOC"; end
    renumberPorts(mdl, outOrder);
    save_system(mdl, fullfile(folder, mdl + ".slx"));
    close_system(mdl);
    registerInProject(fullfile(folder, mdl + ".slx"));
end

% ---------------------------------------------------------------------
function buildHppcSub(sys)
    % HPPC with its bus interface. Every input group (bus inport + Bus
    % Selector) sits level with the HPPC ports it feeds; each link block
    % sits on the line of its port, so all lines are straight.
    newSubsystem(sys);
    hb = "p.ctrl.hppc.Pbase";
    meas = ["gain:1/" + hb, "memory", "delay:p.comm.meas_hppc"];
    pitch = 38;                                   % HPPC port spacing
    add_block("simulink/Ports & Subsystems/Model", sys + "/HPPC", "ModelName", "HPPC", ...
        "Position", [560 40 740 40 + 17*pitch]);
    Simulink.ModelReference.refresh(sys + "/HPPC");
    setArg(sys + "/HPPC", "par", "p.ctrl.hppc.par");
    set_param(sys + "/HPPC", "BackgroundColor", areaColor("controller"));

    % group: inport name, port number, bus elements, HPPC port per element, link steps per element
    G = { ...
      "TSO",       1, ["P_ref","Q_ref","f_meas"],             1:3,   {["gain:1/" + hb, "delay:p.comm.tso@p.grid.Pref/" + hb], ["gain:1/" + hb, "delay:p.comm.tso@0"], strings(1,0)}; ...
      "POC meas",  2, ["P_poc","Q_poc","V_poc"],              4:6,   {meas, meas, ["memory", "delay:p.comm.meas_hppc@1"]}; ...
      "meas wpp",  3, ["P_avail","P","Q"],                    7:9,   {meas, meas, meas}; ...
      "meas spp",  4, ["P_avail","P","Q"],                    10:12, {meas, meas, meas}; ...
      "meas bess", 5, ["P","Q","P_chg_max","P_dis_max","SOC"], 13:17, {meas, meas, meas, meas, ["memory", "delay:p.comm.meas_hppc"]}};
    hppcIn = ["P_ref","Q_ref","f_meas","P_poc","Q_poc","V_poc","P_avail_wpp","P_wpp","Q_wpp", ...
              "P_avail_spp","P_spp","Q_spp","P_chg_max_bess","P_dis_max_bess","SOC_bess","P_bess","Q_bess"];
    for i = 1:size(G, 1)
        [name, port, elems, ports, steps] = G{i, :};
        % BESS: bus order P, Q, P_chg_max, P_dis_max, SOC -> HPPC ports 16, 17, 13, 14, 15
        if name == "meas bess", ports = [16 17 13 14 15]; end
        inport(sys, name, port);
        sel = name + " sel";
        add_block("simulink/Signal Routing/Bus Selector", sys + "/" + sel);
        set_param(sys + "/" + sel, "OutputSignals", char(strjoin(elems, ",")));
        % selector spans the HPPC ports of this group so its outputs line up
        y1 = portY(sys + "/HPPC", "in", min(ports)) - pitch/2;
        set_param(sys + "/" + sel, "Position", [160, y1, 165, y1 + numel(elems)*pitch]);
        rowInto(sys, sel + "/1", name, "", 60);
        % order the selector outputs by HPPC port so lines do not cross
        [~, order] = sort(ports);
        set_param(sys + "/" + sel, "OutputSignals", char(strjoin(elems(order), ",")));
        for j = 1:numel(order)
            e = order(j);  k = ports(e);  st = steps{e};
            if isempty(st)
                ln(sys, sel + "/" + j, "HPPC/" + k, "");
            else
                L = makeLink(sys, hppcIn(k), st);
                rowInto(sys, "HPPC/" + k, L, "", 60);
                ln(sys, sel + "/" + j, L + "/1", "");
            end
        end
    end

    % outputs: set-point buses in W / var, bus creators level with the ports
    tags = ["wpp", "spp", "bess"];
    for i = 1:3
        bc = "set-points " + tags(i) + " bus";
        add_block("simulink/Signal Routing/Bus Creator", sys + "/" + bc, "Inputs", "2");
        y1 = portY(sys + "/HPPC", "out", 2*i-1);  y2 = portY(sys + "/HPPC", "out", 2*i);
        d = y2 - y1;
        set_param(sys + "/" + bc, "Position", [930, y1 - d/2, 935, y2 + d/2]);
        for j = 1:2
            pq = ["P_set", "Q_set"];  unit = [" to W", " to var"];
            L = makeLink(sys, pq(j) + "_" + tags(i) + unit(j), "gain:" + hb);
            rowFrom(sys, "HPPC/" + (2*(i-1) + j), L, "", 50);
            ln(sys, L + "/1", bc + "/" + j, pq(j));
        end
        add_block("simulink/Sinks/Out1", sys + "/set-points " + tags(i), "Port", num2str(i), "Position", [0 0 30 14]);
        rowFrom(sys, bc + "/1", "set-points " + tags(i), "", 50);
    end
    % (the BESS set-point is logged at the top level, from the set-point bus)
    set_param(sys, "Description", "HPP controller (HPPC.slx) with bus interface. Inputs in W / var / pu; " + ...
        "converted to pu on p.ctrl.hppc.Pbase and delayed (p.comm) here. Outputs: set-point buses in W / var.");
    hideStructuralNames(sys);
end

% ---------------------------------------------------------------------
function buildPlantSub(sys, p, pl, idx, offset)
    % One plant, laid out in columns:
    %   bus inputs -> Goto | links -> plant controller -> links -> Goto |
    %   N assets (one row each, inputs via From + Selector) -> Goto |
    %   From -> Mux -> links -> bus outputs
    % In:  1 set-point bus (W/var) from the HPPC, 2 network bus
    % Out: 1 measurement bus (W/var) to the HPPC, 2 current injections
    %      (system pu) to the network. The BESS measurement bus also
    %      carries SOC_plant (for logging).
    newSubsystem(sys);
    N  = p.(pl.t).N;
    nA = p.wt.N + p.pv.N + p.bess.N;
    pb = "p.ctrl." + pl.c + ".Pbase";
    ab = "p.ctrl." + pl.t + ".Pbase";
    sb = "p.grid.Pbase";
    C  = pl.ctrl;
    isB = pl.type == "BESS";
    blue = "[0.592 0.620 0.957]";  green = "[0.122 0.816 0.510]";  orange = "[0.988 0.463 0.204]";

    % ---- plant controller ----
    nCin = 7 + isB;  nCout = 5 + 3*isB;
    pitch = 42;
    add_block("simulink/Ports & Subsystems/Model", sys + "/" + C, "ModelName", C, ...
        "Position", [760 60 940 60 + max(nCin, nCout)*pitch]);
    Simulink.ModelReference.refresh(sys + "/" + C);
    setArg(sys + "/" + C, "par", "p.ctrl." + pl.c + ".par");
    set_param(sys + "/" + C, "BackgroundColor", areaColor("controller"));
    cpos = get_param(sys + "/" + C, "Position");

    % ---- bus inputs -> Goto (left column) ----
    inport(sys, "set-points", 1);
    inport(sys, "network", 2);
    netEl = ["V_re", "V_im", "P_plant", "Q_plant", "V_bus", "f"];
    add_block("simulink/Signal Routing/Bus Selector", sys + "/set-points sel", ...
        "Position", [cpos(1) - 420, cpos(4) + 135, cpos(1) - 415, cpos(4) + 207]);
    add_block("simulink/Signal Routing/Bus Selector", sys + "/network sel", ...
        "Position", [cpos(1) - 420, cpos(4) + 255, cpos(1) - 415, cpos(4) + 435]);
    rowInto(sys, "set-points sel/1", "set-points", "", 60);
    rowInto(sys, "network sel/1", "network", "", 60);
    set_param(sys + "/set-points sel", "OutputSignals", "P_set,Q_set");
    set_param(sys + "/network sel", "OutputSignals", char(strjoin(netEl, ",")));
    for e = ["P_set", "Q_set"]
        rowFrom(sys, "set-points sel/" + find(["P_set", "Q_set"] == e), addGoto(sys, e, blue), "", 40);
    end
    for j = 1:numel(netEl)
        rowFrom(sys, "network sel/" + j, addGoto(sys, netEl(j), green), "", 40);
    end

    % ---- into the plant controller: From -> (Selector) -> link, level with each port ----
    rowInto(sys, C + "/1", [addFrom(sys, "P_set", blue), makeLink(sys, "P_set", ["delay:p.comm.hppc_to_plant", "gain:1/" + pb])], "", 40);
    rowInto(sys, C + "/2", [addFrom(sys, "Q_set", blue), makeLink(sys, "Q_set", ["delay:p.comm.hppc_to_plant", "gain:1/" + pb])], "", 40);
    for j = 1:2
        s = ["P", "Q"]; s = s(j);
        sel = s + " this plant";
        add_block("simulink/Signal Routing/Selector", sys + "/" + sel, "IndexOptions", "Index vector (dialog)", ...
            "Indices", num2str(idx), "InputPortWidth", "3", "Position", [0 0 40 30]);
        rowInto(sys, C + "/" + (2 + j), [addFrom(sys, s + "_plant", green), sel, ...
            makeLink(sys, s + "_meas", ["gain:1/" + pb, "memory", "delay:p.comm.meas_plant"])], "", 40);
    end
    rowInto(sys, C + "/5", [addFrom(sys, "V_bus", green), makeLink(sys, "V_meas", ["memory", "delay:p.comm.meas_plant@1"])], "", 40);
    rowInto(sys, C + "/6", [addFrom(sys, "P_avail_k", orange), ...
        makeLink(sys, "P_avail_k", ["gain:" + ab + "/" + pb, "memory", "delay:p.comm.meas_plant"])], "", 40);
    if isB
        rowInto(sys, C + "/7", [addFrom(sys, "SOC_k", orange), makeLink(sys, "SOC_k", ["memory", "delay:p.comm.meas_plant"])], "", 40);
    end
    % grid frequency, measured locally by the plant controller
    rowInto(sys, C + "/" + nCin, [addFrom(sys, "f", green), makeLink(sys, "f_meas", "memory")], "", 40);

    % ---- out of the plant controller ----
    % set-points to the assets: link -> Goto
    rowFrom(sys, C + "/1", [makeLink(sys, "P_set_k", ["delay:p.comm.plant_to_asset", "gain:" + pb + "/" + ab]), ...
        addGoto(sys, "P_set_k", blue)], "", 40);
    rowFrom(sys, C + "/2", [makeLink(sys, "Q_set_k", ["delay:p.comm.plant_to_asset", "gain:" + pb + "/" + ab]), ...
        addGoto(sys, "Q_set_k", blue)], "", 40);
    % ---- assets: second row, under the plant controller (grows down with N) ----
    x0 = cpos(1) + 40;  rowH = 220;  yTop = cpos(4) + 115;
    outNames = ["P", "Q", "P_avail", "I_re", "I_im", "SOC"];
    nOut = 5 + isB;
    for k = 1:N
        a = sprintf("%s %d", pl.type, k);
        g = offset + k;                                  % global asset index in the network
        ya = yTop + (k-1)*rowH;
        add_block("simulink/Ports & Subsystems/Model", sys + "/" + a, "ModelName", pl.type + "_asset", ...
            "Position", [x0, ya, x0 + 150, ya + 180]);
        Simulink.ModelReference.refresh(sys + "/" + a);
        setArg(sys + "/" + a, "ap", sprintf("p.%s.ap(%d)", pl.t, k));
        set_param(sys + "/" + a, "BackgroundColor", areaColor("asset"));
        feeds = {"P_set_k", k, N, blue; "Q_set_k", k, N, blue; "V_re", g, nA, green; "V_im", g, nA, green};
        for j = 1:4
            [tag, ix, w, col] = feeds{j, :};
            sel = sprintf("%s %d", tag, k);
            add_block("simulink/Signal Routing/Selector", sys + "/" + sel, "IndexOptions", "Index vector (dialog)", ...
                "Indices", num2str(ix), "InputPortWidth", num2str(w), "Position", [0 0 40 30]);
            rowInto(sys, a + "/" + j, [addFrom(sys, tag, col), sel], "", 40);
        end
        switch pl.type
            case "WT"
                src = sprintf("wind %d", k);
                add_block("simulink/Sources/Step", sys + "/" + src, "Time", "p.wt.tStep", ...
                    "Before", "p.wt.Pavail0", "After", "p.wt.PavailStep", "SampleTime", "0", "Position", [0 0 30 30]);
                rowInto(sys, a + "/5", src, "", 140);
            case "PV"
                src = sprintf("irradiance %d", k);
                add_block("simulink/Sources/Constant", sys + "/" + src, "Value", "p.pv.Pavail", "Position", [0 0 70 30]);
                rowInto(sys, a + "/5", src, "", 140);
        end
        % outputs: P, Q unused (terminate); the others to local Gotos tagged <name>_<k>
        for j = 1:nOut
            if j <= 2
                t = sprintf("%s %s unused", a, outNames(j));
                add_block("simulink/Sinks/Terminator", sys + "/" + t, "Position", [0 0 20 20]);
                rowFrom(sys, a + "/" + j, t, "", 40);
            else
                col = orange;  if startsWith(outNames(j), "I_"), col = green; end
                rowFrom(sys, a + "/" + j, addGoto(sys, sprintf("%s_%d", outNames(j), k), col), "", 40);
            end
        end
    end

    % ---- right column: From -> Mux -> link -> outputs ----
    xm = cpos(3) + 620;  ym = yTop;              % right of the assets, in the output column
    muxed = outNames(3:nOut);
    for j = 1:numel(muxed)
        mx = muxed(j) + "_k";
        col = orange;  if startsWith(muxed(j), "I_"), col = green; end
        add_block("simulink/Signal Routing/Mux", sys + "/" + mx, "Inputs", num2str(N), ...
            "Position", [xm, ym, xm + 5, ym + N*40]);
        for k = 1:N
            rowInto(sys, mx + "/" + k, addFrom(sys, sprintf("%s_%d", muxed(j), k), col), "", 40);
        end
        ym = ym + N*40 + 30;
    end
    rowFrom(sys, "P_avail_k/1", addGoto(sys, "P_avail_k", orange), "P_avail_k", 60);
    for j = 1:2
        s = ["I_re", "I_im"]; s = s(j);
        L = makeLink(sys, s + " to system pu", "gain:" + ab + "/" + sb);
        rowFrom(sys, s + "_k/1", L, s + "_k", 60);
    end
    % bus creator spanning both links (inputs level with the link outputs)
    y1 = portY(sys + "/I_re to system pu link", "out", 1);  y2 = portY(sys + "/I_im to system pu link", "out", 1);
    lp = get_param(sys + "/I_re to system pu link", "Position");
    add_block("simulink/Signal Routing/Bus Creator", sys + "/injection", "Inputs", "2", ...
        "Position", [lp(3) + 80, y1 - (y2-y1)/2, lp(3) + 85, y2 + (y2-y1)/2]);
    ln(sys, "I_re to system pu link/1", "injection/1", "I_re");
    ln(sys, "I_im to system pu link/1", "injection/2", "I_im");
    add_block("simulink/Sinks/Out1", sys + "/I", "Port", "2", "Position", [0 0 30 14]);
    rowFrom(sys, "injection/1", "I", "", 60);
    if isB
        rowFrom(sys, "SOC_k/1", addGoto(sys, "SOC_k", orange), "SOC_k", 60);
    end

    % ---- measurements to the HPPC (output column, level with the controller ports):
    %      link (to W) -> bus creator -> outport, in line with the I output
    if isB
        add_block("simulink/Sinks/Terminator", sys + "/P_avail unused", "Position", [0 0 20 20]);
        rowFrom(sys, C + "/3", "P_avail unused", "P_avail", 40);
        mPorts = 4:8;  names = ["P", "Q", "P_chg_max", "P_dis_max", "SOC"];
    else
        mPorts = 3:5;  names = ["P_avail", "P", "Q"];
    end
    % BESS: one extra element below the controller ports, SOC_plant = the
    % share-weighted mean of the asset SOCs (continuous; for logging), so
    % all three plants have the same two outputs
    nBus = numel(names) + isB;
    add_block("simulink/Signal Routing/Bus Creator", sys + "/meas bus", "Inputs", num2str(nBus));
    y1 = portY(sys + "/" + C, "out", mPorts(1));
    d = portY(sys + "/" + C, "out", 2) - portY(sys + "/" + C, "out", 1);   % actual output spacing
    xb = get_param(sys + "/injection", "Position");  xb = xb(1);   % same column as the I output
    set_param(sys + "/meas bus", "Position", [xb, y1 - d/2, xb + 5, y1 - d/2 + nBus*d]);
    for j = 1:numel(names)
        if names(j) == "SOC"
            ln(sys, C + "/" + mPorts(j), "meas bus/" + j, "SOC");
        else
            L = makeLink(sys, names(j) + " to W", "gain:" + pb);
            lw = get_param(sys + "/" + L, "Position");  lw = lw(3) - lw(1);
            px = get_param(sys + "/" + C, "Position");
            rowFrom(sys, C + "/" + mPorts(j), L, "", xb - 60 - lw - px(3));   % link right before the bus
            ln(sys, L + "/1", "meas bus/" + j, names(j));
        end
    end
    if isB
        add_block("simulink/Math Operations/Gain", sys + "/SOC plant", ...
            "Gain", "p.ctrl.bessc.par.share'", "Multiplication", "Matrix(K*u)", "Position", [0 0 60 30]);
        rowInto(sys, "meas bus/" + nBus, [addFrom(sys, "SOC_k", orange), "SOC plant"], "SOC_plant", 40);
    end
    add_block("simulink/Sinks/Out1", sys + "/meas", "Port", "1", "Position", [0 0 30 14]);
    rowFrom(sys, "meas bus/1", "meas", "", 60);


    % ---- background areas by role ----
    xSplit = cpos(3) + 440;                      % controller column | output column
    hI = addArea(sys, "Bus inputs", [-inf, cpos(4) + 20, x0 - 330, inf], areaColor("interface"));
    hC = addArea(sys, C + " plant controller (future DLL)", [-inf, -inf, xSplit, cpos(4) + 20], areaColor("controller"));
    hA = addArea(sys, N + " × " + pl.type + " asset", [x0 - 330, yTop - 10, xSplit, inf], areaColor("asset"));
    hO = addArea(sys, "Outputs to HPPC and network", [xSplit, -inf, inf, inf], areaColor("interface"));
    alignAreaGrid(hI, hC, hA, hO);

    set_param(sys, "Description", pl.title + ": " + C + " (plant controller) and " + N + " " + pl.type + ...
        " asset instances. Bus interface in W / var; per-unit conversions and the delays of the links into " + ...
        "this plant (p.comm) are inside. Local Goto/From: blue = set-points, green = network, orange = " + ...
        "asset signals back to the plant controller.");
    hideStructuralNames(sys);
end

%% ===================================================================
%  Layout helpers: rows of blocks level with a port, local Goto/From,
%  link blocks
%  ===================================================================
function alignAreaGrid(hI, hC, hA, hO)
    % Align the four plant areas in a grid:
    %   [ controller              ] [ outputs ]
    %   [ bus inputs ] [ assets   ] [         ]
    % shared left edge (inputs, controller), shared right edge (controller,
    % assets), shared top (controller, outputs), shared top of the lower row
    % (inputs, assets), shared bottom (inputs, assets, outputs)
    if any([hI hC hA hO] == -1), return, end
    I = get_param(hI, "Position");  Cc = get_param(hC, "Position");
    A = get_param(hA, "Position");  O = get_param(hO, "Position");
    left = min(I(1), Cc(1));  rightL = max(Cc(3), A(3));
    top = min(Cc(2), O(2));   rowTop = min(I(2), A(2));
    bottom = max([I(4), A(4), O(4)]);
    set_param(hI, "Position", [left, rowTop, I(3), bottom]);
    set_param(hA, "Position", [A(1), rowTop, rightL, bottom]);
    set_param(hC, "Position", [left, top, rightL, Cc(4)]);
    set_param(hO, "Position", [O(1), top, O(3), bottom]);
end

function c = areaColor(role)
    % light background tints by role (DTU palette, very light)
    switch role
        case "controller", c = "[0.894 0.906 0.988]";   % blue: controllers / future DLLs
        case "asset",      c = "[0.878 0.965 0.925]";   % green: asset (plant-side) models
        otherwise,         c = "[0.950 0.950 0.950]";   % grey: interfaces, network
    end
end

function h = addArea(sys, title, region, color)
    % Area annotation fitted around all blocks whose centre lies in region
    % [left top right bottom] (use -inf / inf for open edges), with a title
    b = find_system(sys, "SearchDepth", 1, "Type", "Block");
    box = [inf inf -inf -inf];
    for i = 2:numel(b)
        p = get_param(b{i}, "Position");
        c = [(p(1) + p(3))/2, (p(2) + p(4))/2];
        if c(1) >= region(1) && c(1) <= region(3) && c(2) >= region(2) && c(2) <= region(4)
            box = [min(box(1), p(1)), min(box(2), p(2)), max(box(3), p(3)), max(box(4), p(4))];
        end
    end
    h = -1;
    if any(isinf(box)), return, end
    h = add_block("built-in/Area", sys + "/" + title, ...
        "Position", box + [-12 -36 12 22]);      % room for the title and block names
    set_param(h, "BackgroundColor", color);
end

function renumberPorts(sys, names)
    % Give the named Inports (or Outports) the port numbers 1..n in this
    % order. Needed when ports are created out of order: Simulink shifts
    % existing numbers when a port number is requested.
    for k = 1:numel(names)
        set_param(sys + "/" + names(k), "Port", num2str(k));
    end
end

function hideStructuralNames(sys)
    % Bus Selectors / Creators, Muxes and Selectors carry no information in
    % their names (the signal labels say it); hiding them removes clutter
    types = ["BusSelector", "BusCreator", "Mux", "Demux", "Selector", "Terminator"];
    b = find_system(sys, "SearchDepth", 1, "Type", "Block");
    for i = 2:numel(b)   % 1 is the system itself
        if any(string(get_param(b{i}, "BlockType")) == types)
            set_param(b{i}, "ShowName", "off");
        end
    end
end

function y = portY(blk, kind, k)
    ph = get_param(blk, "PortHandles");
    if kind == "in", h = ph.Inport(k); else, h = ph.Outport(k); end
    pos = get_param(h, "Position");
    y = pos(2);
end

function place(blk, x, yc)
    % move a block so its left edge is at x and its centre at height yc
    pos = get_param(blk, "Position");
    w = pos(3) - pos(1);  h = pos(4) - pos(2);
    set_param(blk, "Position", round([x, yc - h/2, x + w, yc + h/2]));
end

function rowInto(sys, dst, blocks, lineName, gap)
    % Place blocks (left to right) on one line ending at input port dst
    % ("Block/k"), all centred on the port's height, and connect them:
    % blocks(1) -> blocks(2) -> ... -> dst. Straight lines.
    blocks = string(blocks);
    dBlk = extractBefore(dst, "/");  k = str2double(extractAfter(dst, "/"));
    ph = get_param(sys + "/" + dBlk, "PortHandles");
    pp = get_param(ph.Inport(k), "Position");
    x = pp(1) - gap;
    for i = numel(blocks):-1:1
        b = sys + "/" + blocks(i);
        pos = get_param(b, "Position");  w = pos(3) - pos(1);
        place(b, x - w, pp(2));
        x = x - w - 30;
    end
    for i = 1:numel(blocks) - 1
        ln(sys, blocks(i) + "/1", blocks(i+1) + "/1", "");
    end
    ln(sys, blocks(end) + "/1", dst, lineName);
end

function rowFrom(sys, src, blocks, lineName, gap)
    % Place blocks (left to right) on one line starting at output port src,
    % centred on the port's height, and connect src -> blocks(1) -> ...
    blocks = string(blocks);
    sBlk = extractBefore(src, "/");  k = str2double(extractAfter(src, "/"));
    ph = get_param(sys + "/" + sBlk, "PortHandles");
    pp = get_param(ph.Outport(k), "Position");
    x = pp(1) + gap;
    for i = 1:numel(blocks)
        b = sys + "/" + blocks(i);
        pos = get_param(b, "Position");  w = pos(3) - pos(1);
        place(b, x, pp(2));
        x = x + w + 30;
    end
    ln(sys, src, blocks(1) + "/1", lineName);
    for i = 1:numel(blocks) - 1
        ln(sys, blocks(i) + "/1", blocks(i+1) + "/1", "");
    end
end

function name = addGoto(sys, tag, color)
    % local Goto (visible only in this system); returns the block name
    name = tag + " goto";
    add_block("simulink/Signal Routing/Goto", sys + "/" + name, "GotoTag", tag, ...
        "TagVisibility", "local", "BackgroundColor", color, "ShowName", "off", ...
        "Position", [0 0 12 + 8*strlength(tag) 22]);
end

function name = addFrom(sys, tag, color)
    % From for a local Goto; returns the (unique) block name
    i = 1;
    while getSimulinkBlockHandle(sys + "/" + sprintf("%s from %d", tag, i)) ~= -1
        i = i + 1;
    end
    name = sprintf("%s from %d", tag, i);
    add_block("simulink/Signal Routing/From", sys + "/" + name, "GotoTag", tag, ...
        "BackgroundColor", color, "ShowName", "off", "Position", [0 0 12 + 8*strlength(tag) 22]);
end

function name = makeLink(sys, label, steps)
    % Compact "link" block: masked subsystem with the steps in series.
    % steps: "gain:<expr>" (unit / per-unit conversion), "memory" (one-step
    % measurement memory), "delay:<time>[@<initial output>]" (communication
    % delay). Returns the block name "<label> link" (not yet connected).
    name = label + " link";
    blk = sys + "/" + name;
    add_block("simulink/Ports & Subsystems/Subsystem", blk);
    delete_line(blk, "In1/1", "Out1/1");
    prev = "In1/1";
    lbl = strings(0, 1);  desc = strings(0, 1);
    for i = 1:numel(steps)
        s = steps(i);
        kind = extractBefore(s + ":", ":");
        arg  = extractAfter(s, ":");
        bn = kind + " " + i;
        switch kind
            case "gain"
                add_block("simulink/Math Operations/Gain", blk + "/" + bn, "Gain", arg);
                if startsWith(arg, "1/")
                    lbl(end+1) = "to pu"; %#ok<AGROW>    % SI -> per unit
                elseif contains(arg, "/")
                    lbl(end+1) = "pu base"; %#ok<AGROW>  % per unit -> other per-unit base
                else
                    lbl(end+1) = "to SI"; %#ok<AGROW>    % per unit -> W / var
                end
                desc(end+1) = "gain " + arg; %#ok<AGROW>
            case "delay"
                parts = split(arg, "@");
                init = "0";
                if numel(parts) > 1, init = parts(2); end
                add_block("simulink/Continuous/Transport Delay", blk + "/" + bn, ...
                    "DelayTime", parts(1), "InitialOutput", init);
                lbl(end+1) = erase(parts(1), "p."); %#ok<AGROW>
                desc(end+1) = "transport delay " + parts(1) + " (initial output " + init + ")"; %#ok<AGROW>
            case "memory"
                add_block("simulink/Discrete/Memory", blk + "/" + bn);
                lbl(end+1) = "mem"; %#ok<AGROW>
                desc(end+1) = "one-step memory"; %#ok<AGROW>
        end
        add_line(blk, prev, bn + "/1", "autorouting", "smart");
        prev = bn + "/1";
    end
    add_line(blk, prev, "Out1/1", "autorouting", "smart");
    Simulink.BlockDiagram.arrangeSystem(blk);
    mk = Simulink.Mask.create(blk);
    mk.Display = "disp('" + strjoin(lbl, " | ") + "')";
    mk.Description = "Link: " + strjoin(desc, ", then ") + ".";
    mk.IconOpaque = "opaque";
    % the icon says what the link does and the port it feeds says which
    % signal it carries, so the name is hidden (keeps rows compact)
    set_param(blk, "Position", [0 0 30 + 7*strlength(strjoin(lbl, " | ")) 26], "ShowName", "off");
end

function chain(sys, src, steps, dst, label, outName)
    % link block from src to dst (used where alignment does not matter)
    arguments
        sys; src; steps; dst; label
        outName = ""
    end
    L = makeLink(sys, label, steps);
    ln(sys, src, L + "/1", label);
    ln(sys, L + "/1", dst, outName);
end

%% ===================================================================
%  WT_ctrl template: IEC 61400-27-1 Type 4A control as a block diagram
%  (stage 3). Structure after IEC 61400-27-1 (measurement, P control type
%  4A, Q control, current limitation); block details and parameter values
%  to be checked against the standard. Ports unchanged:
%    in  1 P_set  2 Q_set  3 V_term  4 P_meas  5 Q_meas  6 P_wind_avail
%    out 1 ip_cmd 2 iq_cmd 3 P_avail          (pu on the asset rating)
%  ===================================================================
function buildWTCtrlType4A(folder)
    mdl = "WT_ctrl";
    closeIfLoaded(mdl);
    new_system(mdl);
    set_param(mdl, "SolverType", "Fixed-step", "Solver", "FixedStepDiscrete", ...
        "FixedStep", "Ts", "StopTime", "inf", "Description", ...
        "WT asset controller: IEC 61400-27-1 Type 4A control (measurement, P control type 4A, Q control " + ...
        "with closed-loop reactive power control (MqG) and dip current injection, current limitation). Per unit on the asset rating. Parameters: " + ...
        "model argument par (= p.ctrl.wt.par, from p.wt.iec.ctrl). Structure after the standard; " + ...
        "details and values to be checked against it.");
    ws = get_param(mdl, "ModelWorkspace");
    ws.DataSource = "MATLAB Code";
    ws.MATLABCode = char(strjoin([
        "% Defaults for standalone use and code generation, from HPP_parameters."
        "% In HPP.slx the Model block passes p.ctrl.wt.par as the argument ""par""."
        "pDefault = HPP_parameters();"
        "Ts  = pDefault.ctrl.wt.Ts;    % [s] fixed sample time"
        "par = pDefault.ctrl.wt.par;   % tunable parameters (model argument)"
        "clear pDefault"], newline));
    reload(ws);
    set_param(mdl, "ParameterArgumentNames", "par");
    B = @(lib, name, pos, varargin) add_block(lib, mdl + "/" + name, "Position", pos, varargin{:});
    L = @(a, b) add_line(mdl, a, b, "autorouting", "smart");

    % ---- inports ----
    ins = ["P_set", "Q_set", "V_term", "P_meas", "Q_meas", "P_wind_avail"];
    yIn = [100, 400, 260, 600, 650, 160];
    for k = 1:6
        B("simulink/Sources/In1", ins(k), [30, yIn(k)-7, 60, yIn(k)+7], "Port", num2str(k), ...
            "OutDataTypeStr", "double", "PortDimensions", "1", "SampleTime", "Ts");
    end

    % ---- measurement: first-order filters, dip detection ----
    % first-order lag y(k) = p*y(k-1) + (1-p)*u(k), p = 1 - Ts/T, with an
    % initial output (the voltage filter starts at 1 pu, not 0)
    lag = @(name, T, y, y0) B("simulink/Discrete/Transfer Fcn First Order", name, [140, y-17, 230, y+17], ...
        "PoleZ", "1 - Ts/" + T, "ICPrevOutput", y0);
    lag("U filt", "par.Tufilt", 260, "1");  L("V_term/1", "U filt/1");
    lag("P filt", "par.Tpfilt", 600, "0");  L("P_meas/1", "P filt/1");
    lag("Q filt", "par.Tqfilt", 650, "0");  L("Q_meas/1", "Q filt/1");
    B("simulink/Sinks/Terminator", "P filt unused", [270, 590, 290, 610]);   % no P loop in Type 4A
    L("P filt/1", "P filt unused/1");
    B("simulink/Sources/Constant", "udip", [260, 200, 330, 224], "Value", "par.udip");
    B("simulink/Logic and Bit Operations/Relational Operator", "dip", [380, 205, 420, 255], "Operator", "<");
    L("U filt/1", "dip/1");  L("udip/1", "dip/2");
    B("simulink/Sources/Constant", "Vmin", [260, 300, 330, 324], "Value", "par.Vmin");
    B("simulink/Math Operations/MinMax", "U floor", [380, 265, 420, 315], "Function", "max", "Inputs", "2");
    L("U filt/1", "U floor/1");  L("Vmin/1", "U floor/2");

    % ---- P control type 4A ----
    B("simulink/Sources/Constant", "zero", [140, 40, 180, 64], "Value", "0");
    B("simulink/Discontinuities/Saturation Dynamic", "P_ref limit", [260, 60, 320, 170]);
    L("P_wind_avail/1", "P_ref limit/1");  L("P_set/1", "P_ref limit/2");  L("zero/1", "P_ref limit/3");
    B("simulink/Discontinuities/Rate Limiter", "dpmaxp4A", [380, 100, 430, 130], ...
        "RisingSlewLimit", "par.dpmaxp4A", "FallingSlewLimit", "-par.dpmaxp4A");
    L("P_ref limit/1", "dpmaxp4A/1");
    B("simulink/Discrete/Transfer Fcn First Order", "Tpordp4A", [480, 98, 570, 132], ...
        "PoleZ", "1 - Ts/par.Tpordp4A", "ICPrevOutput", "0");
    L("dpmaxp4A/1", "Tpordp4A/1");
    B("simulink/Math Operations/Divide", "p_ord to ip", [640, 100, 690, 150]);
    L("Tpordp4A/1", "p_ord to ip/1");  L("U floor/1", "p_ord to ip/2");

    % ---- Q control (open-loop reactive power + dip current injection) ----
    B("simulink/Math Operations/Divide", "q_ref to iq", [640, 380, 690, 430]);
    L("Q_set/1", "q_ref to iq/1");  L("U floor/1", "q_ref to iq/2");
    % reactive power control mode MqG = 1: PI (kpq, kiq) on q_ref - q_filt,
    % frozen during a dip, added to the open-loop current q_ref / U
    % (MqG = 2: open-loop reactive power control only)
    B("simulink/Math Operations/Sum", "q error", [380, 560, 410, 600], "Inputs", "+-");
    L("Q_set/1", "q error/1");  L("Q filt/1", "q error/2");
    B("simulink/Sources/Constant", "MqG = 1", [380, 620, 470, 640], "Value", "double(par.MqG == 1)");
    B("simulink/Sources/Constant", "no q loop", [480, 520, 510, 540], "Value", "0");
    B("simulink/Signal Routing/Switch", "q loop on?", [520, 555, 560, 615], "Criteria", "u2 ~= 0");
    L("q error/1", "q loop on?/1");  L("MqG = 1/1", "q loop on?/2");  L("no q loop/1", "q loop on?/3");
    B("simulink/Signal Routing/Switch", "freeze in dip", [600, 555, 640, 615], "Criteria", "u2 ~= 0");
    L("no q loop/1", "freeze in dip/1");  L("dip/1", "freeze in dip/2");  L("q loop on?/1", "freeze in dip/3");
    B("simulink/Discrete/Discrete PID Controller", "kpq kiq", [680, 560, 760, 610], "Controller", "PI", ...
        "P", "par.kpq", "I", "par.kiq", "LimitOutput", "on", "UpperSaturationLimit", "par.imax", ...
        "LowerSaturationLimit", "-par.imax", "AntiWindupMode", "clamping", "SampleTime", "-1");
    L("freeze in dip/1", "kpq kiq/1");
    B("simulink/Math Operations/Sum", "iq ff + PI", [710, 390, 730, 440], "Inputs", "++");
    L("q_ref to iq/1", "iq ff + PI/1");  L("kpq kiq/1", "iq ff + PI/2");
    B("simulink/Sources/Constant", "uref0", [260, 470, 330, 494], "Value", "par.uref0");
    B("simulink/Math Operations/Sum", "uref0 - U", [380, 465, 410, 505], "Inputs", "+-");
    L("uref0/1", "uref0 - U/1");  L("U filt/1", "uref0 - U/2");
    B("simulink/Math Operations/Gain", "Kqv", [470, 470, 520, 500], "Gain", "par.Kqv");
    L("uref0 - U/1", "Kqv/1");
    B("simulink/Math Operations/Sum", "iq + dip", [760, 440, 790, 480], "Inputs", "++");
    L("iq ff + PI/1", "iq + dip/1");  L("Kqv/1", "iq + dip/2");
    B("simulink/Signal Routing/Switch", "in dip?", [850, 395, 890, 455], "Criteria", "u2 ~= 0");
    L("iq + dip/1", "in dip?/1");  L("dip/1", "in dip?/2");  L("iq ff + PI/1", "in dip?/3");
    B("simulink/Discontinuities/Saturation", "iqH1", [950, 410, 990, 440], ...
        "UpperLimit", "par.iqH1", "LowerLimit", "-par.iqH1");
    L("in dip?/1", "iqH1/1");

    % ---- current limitation: P priority normally, Q priority in a dip ----
    body = [
        "% IEC 61400-27-1 current limitation (simplified): total current <= imax;"
        "% normal operation: active current first, reactive current gets the rest;"
        "% during a voltage dip: reactive current first."
        "imax = par.imax;"
        "if in_dip"
        "    iq_cmd = min(max(iq_req, -imax), imax);"
        "    ipmax  = sqrt(max(imax^2 - iq_cmd^2, 0));"
        "    ip_cmd = min(max(ip_req, 0), ipmax);"
        "else"
        "    ip_cmd = min(max(ip_req, 0), imax);"
        "    iqmax  = sqrt(max(imax^2 - ip_cmd^2, 0));"
        "    iq_cmd = min(max(iq_req, -iqmax), iqmax);"
        "end"];
    addMatlabFunction(mdl + "/current limitation", "current_limitation", ["ip_req", "iq_req", "in_dip"], ...
        ["ip_cmd", "iq_cmd"], body, "par", [1080, 100, 1240, 460]);
    L("p_ord to ip/1", "current limitation/1");
    L("iqH1/1", "current limitation/2");
    L("dip/1", "current limitation/3");

    % ---- outports ----
    B("simulink/Sinks/Out1", "ip_cmd",  [1320, 153, 1350, 167], "Port", "1", "OutDataTypeStr", "double");
    B("simulink/Sinks/Out1", "iq_cmd",  [1320, 393, 1350, 407], "Port", "2", "OutDataTypeStr", "double");
    B("simulink/Sinks/Out1", "P_avail", [1320,  13, 1350,  27], "Port", "3", "OutDataTypeStr", "double");
    L("current limitation/1", "ip_cmd/1");
    L("current limitation/2", "iq_cmd/1");
    L("P_wind_avail/1", "P_avail/1");   % Type 4A: constant aerodynamic power

    % ---- role areas ----
    addArea(mdl, "Measurement", [100, 180, 440, 340], areaColor("interface"));
    addArea(mdl, "P control type 4A", [130, 0, 700, 175], areaColor("controller"));
    addArea(mdl, "Q control (MqG 1 closed loop, 2 open loop) + dip current", [250, 360, 1000, 640], areaColor("controller"));
    addArea(mdl, "Current limitation", [1050, 50, 1260, 500], areaColor("asset"));

    save_system(mdl, fullfile(folder, mdl + ".slx"));
    close_system(mdl);
    registerInProject(fullfile(folder, mdl + ".slx"));
end

%% ===================================================================
%  PV_ctrl / BESS_ctrl template: WECC second-generation electrical
%  control as a block diagram (stage 4). PV: REEC_B; BESS: REEC_C (REEC_B
%  structure plus state-of-charge limits). Qflag = 0 (reactive current
%  from the Q set-point), no voltage-dependent current limit (VDL), no
%  state freeze during a dip. Ports unchanged:
%    in  1 P_set  2 Q_set  3 V_term  4 P_meas  5 Q_meas  6 P_pv_avail | SOC
%    out 1 ip_cmd 2 iq_cmd 3 P_avail          (pu on the asset rating)
%  ===================================================================
function buildReecCtrl(type, folder)
    isB = type == "BESS";
    mdl = type + "_ctrl";  c = lower(type);
    if isB
        std = "REEC_C"; src = "SOC"; extra = ", state-of-charge limits";
    else
        std = "REEC_B"; src = "P_pv_avail"; extra = "";
    end
    closeIfLoaded(mdl);
    new_system(mdl);
    set_param(mdl, "SolverType", "Fixed-step", "Solver", "FixedStepDiscrete", ...
        "FixedStep", "Ts", "StopTime", "inf", "Description", ...
        type + " asset controller: WECC " + std + " electrical control (voltage filter and dip " + ...
        "detection, P path with ramp and Tpord lag, Q path with Qflag (closed-loop Q PI) and dip current injection " + ...
        "Kqv, current limit logic with PQflag" + extra + "). Per unit on the asset rating. " + ...
        "Parameters: model argument par (= p.ctrl." + c + ".par, from p." + c + ".wecc.ctrl). " + ...
        "Simplified; details and values to be checked against the WECC specification.");
    ws = get_param(mdl, "ModelWorkspace");
    ws.DataSource = "MATLAB Code";
    ws.MATLABCode = char(strjoin([
        "% Defaults for standalone use and code generation, from HPP_parameters."
        "% In HPP.slx the Model block passes p.ctrl." + c + ".par as the argument ""par""."
        "pDefault = HPP_parameters();"
        "Ts  = pDefault.ctrl." + c + ".Ts;    % [s] fixed sample time"
        "par = pDefault.ctrl." + c + ".par;   % tunable parameters (model argument)"
        "clear pDefault"], newline));
    reload(ws);
    set_param(mdl, "ParameterArgumentNames", "par");
    B = @(lib, name, pos, varargin) add_block(lib, mdl + "/" + name, "Position", pos, varargin{:});
    L = @(a, b) add_line(mdl, a, b, "autorouting", "smart");
    lag = @(name, T, pos, y0) B("simulink/Discrete/Transfer Fcn First Order", name, pos, ...
        "PoleZ", "1 - Ts/" + T, "ICPrevOutput", y0);

    % ---- inports ----
    ins = ["P_set", "Q_set", "V_term", "P_meas", "Q_meas", src];
    yIn = [100, 400, 260, 640, 690, 160];
    for k = 1:6
        B("simulink/Sources/In1", ins(k), [30, yIn(k)-7, 60, yIn(k)+7], "Port", num2str(k), ...
            "OutDataTypeStr", "double", "PortDimensions", "1", "SampleTime", "Ts");
    end
    B("simulink/Sinks/Terminator", "P_meas unused", [140, 630, 160, 650]);   % no P loop in REEC_B/C
    L("P_meas/1", "P_meas unused/1");

    % ---- voltage filter Trv, dip detection (V < Vdip or V > Vup) ----
    lag("Vt filt", "par.Trv", [140, 243, 230, 277], "1");  L("V_term/1", "Vt filt/1");
    B("simulink/Sources/Constant", "Vdip", [260, 190, 330, 210], "Value", "par.Vdip");
    B("simulink/Logic and Bit Operations/Relational Operator", "V < Vdip", [380, 190, 420, 230], "Operator", "<");
    L("Vt filt/1", "V < Vdip/1");  L("Vdip/1", "V < Vdip/2");
    B("simulink/Sources/Constant", "Vup", [260, 230, 330, 250], "Value", "par.Vup");
    B("simulink/Logic and Bit Operations/Relational Operator", "V > Vup", [380, 240, 420, 280], "Operator", ">");
    L("Vt filt/1", "V > Vup/1");  L("Vup/1", "V > Vup/2");
    B("simulink/Logic and Bit Operations/Logical Operator", "Voltage_dip", [470, 210, 500, 260], "Operator", "OR");
    L("V < Vdip/1", "Voltage_dip/1");  L("V > Vup/1", "Voltage_dip/2");
    B("simulink/Sources/Constant", "Vmin", [260, 300, 330, 320], "Value", "par.Vmin");
    B("simulink/Math Operations/MinMax", "V floor", [380, 290, 420, 330], "Function", "max", "Inputs", "2");
    L("Vt filt/1", "V floor/1");  L("Vmin/1", "V floor/2");

    % ---- P path: limits, ramp dPmax/dPmin, Tpord, Ipcmd = Pord / V ----
    if isB
        B("simulink/Sources/Constant", "Pmax", [140, 130, 190, 150], "Value", "par.Pmax");
        B("simulink/Sources/Constant", "Pmin", [140, 40, 190, 60], "Value", "-par.Pmax");
        upSrc = "Pmax/1";
    else
        B("simulink/Sources/Constant", "Pmin", [140, 40, 190, 60], "Value", "0");
        upSrc = src + "/1";
    end
    B("simulink/Discontinuities/Saturation Dynamic", "Pmax Pmin", [260, 60, 320, 170]);
    L(upSrc, "Pmax Pmin/1");  L("P_set/1", "Pmax Pmin/2");  L("Pmin/1", "Pmax Pmin/3");
    B("simulink/Discontinuities/Rate Limiter", "dPmax dPmin", [380, 100, 430, 130], ...
        "RisingSlewLimit", "par.dPmax", "FallingSlewLimit", "par.dPmin");
    L("Pmax Pmin/1", "dPmax dPmin/1");
    lag("Tpord", "par.Tpord", [480, 98, 570, 132], "0");
    L("dPmax dPmin/1", "Tpord/1");
    B("simulink/Math Operations/Divide", "Pord to Ip", [640, 100, 690, 150]);
    L("Tpord/1", "Pord to Ip/1");  L("V floor/1", "Pord to Ip/2");

    % ---- Q path (Qflag = 0): Iq = Qext / V, lag Tiq; Iqinj in a dip ----
    B("simulink/Math Operations/Divide", "Qext to Iq", [380, 380, 430, 430]);
    L("Q_set/1", "Qext to Iq/1");  L("V floor/1", "Qext to Iq/2");
    % Qflag = 1: PI (Kqp, Kqi) on Q_set - Q_gen, frozen during a dip, added
    % to the open-loop current Qext / V (Qflag = 0: open loop only)
    B("simulink/Math Operations/Sum", "Q error", [380, 600, 410, 640], "Inputs", "+-");
    L("Q_set/1", "Q error/1");  L("Q_meas/1", "Q error/2");
    B("simulink/Sources/Constant", "Qflag", [380, 660, 430, 680], "Value", "par.Qflag");
    B("simulink/Sources/Constant", "no Q loop", [450, 560, 480, 580], "Value", "0");
    B("simulink/Signal Routing/Switch", "Q loop on?", [480, 600, 520, 660], "Criteria", "u2 ~= 0");
    L("Q error/1", "Q loop on?/1");  L("Qflag/1", "Q loop on?/2");  L("no Q loop/1", "Q loop on?/3");
    B("simulink/Signal Routing/Switch", "freeze in dip", [560, 600, 600, 660], "Criteria", "u2 ~= 0");
    L("no Q loop/1", "freeze in dip/1");  L("Voltage_dip/1", "freeze in dip/2");  L("Q loop on?/1", "freeze in dip/3");
    B("simulink/Discrete/Discrete PID Controller", "Kqp Kqi", [640, 605, 720, 655], "Controller", "PI", ...
        "P", "par.Kqp", "I", "par.Kqi", "LimitOutput", "on", "UpperSaturationLimit", "par.Imax", ...
        "LowerSaturationLimit", "-par.Imax", "AntiWindupMode", "clamping", "SampleTime", "-1");
    L("freeze in dip/1", "Kqp Kqi/1");
    B("simulink/Math Operations/Sum", "ff + PI", [445, 385, 465, 435], "Inputs", "++");
    L("Qext to Iq/1", "ff + PI/1");  L("Kqp Kqi/1", "ff + PI/2");
    lag("Tiq", "par.Tiq", [490, 388, 570, 422], "0");
    L("ff + PI/1", "Tiq/1");
    B("simulink/Sources/Constant", "Vref0", [260, 480, 330, 500], "Value", "par.Vref0");
    B("simulink/Math Operations/Sum", "Vref0 - V", [380, 470, 410, 510], "Inputs", "+-");
    L("Vref0/1", "Vref0 - V/1");  L("Vt filt/1", "Vref0 - V/2");
    B("simulink/Discontinuities/Dead Zone", "dbd1 dbd2", [450, 475, 500, 505], ...
        "LowerValue", "par.dbd1", "UpperValue", "par.dbd2");
    L("Vref0 - V/1", "dbd1 dbd2/1");
    B("simulink/Math Operations/Gain", "Kqv", [530, 475, 570, 505], "Gain", "par.Kqv");
    L("dbd1 dbd2/1", "Kqv/1");
    B("simulink/Discontinuities/Saturation", "Iqh1 Iql1", [610, 475, 650, 505], ...
        "UpperLimit", "par.Iqh1", "LowerLimit", "par.Iql1");
    L("Kqv/1", "Iqh1 Iql1/1");
    B("simulink/Sources/Constant", "no Iqinj", [610, 530, 650, 550], "Value", "0");
    B("simulink/Signal Routing/Switch", "Iqinj in dip", [700, 470, 740, 550], "Criteria", "u2 ~= 0");
    L("Iqh1 Iql1/1", "Iqinj in dip/1");  L("Voltage_dip/1", "Iqinj in dip/2");  L("no Iqinj/1", "Iqinj in dip/3");
    B("simulink/Math Operations/Sum", "Iq + Iqinj", [800, 400, 830, 440], "Inputs", "++");
    L("Tiq/1", "Iq + Iqinj/1");  L("Iqinj in dip/1", "Iq + Iqinj/2");

    % ---- current limit logic ----
    body = [
        "% WECC current limit logic (without VDL tables): |I| <= Imax;"
        "% PQflag = 0: Q priority, PQflag = 1: P priority."];
    if isB
        body = [body
            "% REEC_C: no discharge at SOC <= SOCmin, no charge at SOC >= SOCmax."
            "ipmin_soc = -par.Imax;  ipmax_soc = par.Imax;"
            "if SOC <= par.SOCmin, ipmax_soc = 0; end"
            "if SOC >= par.SOCmax, ipmin_soc = 0; end"];
    else
        body = [body; "ipmin_soc = 0;  ipmax_soc = par.Imax;   % PV: no absorption"];
    end
    body = [body
        "Imax = par.Imax;"
        "if par.PQflag == 0   % Q priority"
        "    iq_cmd = min(max(iq_req, -Imax), Imax);"
        "    ipmax  = sqrt(max(Imax^2 - iq_cmd^2, 0));"
        "    ip_cmd = min(max(ip_req, max(-ipmax, ipmin_soc)), min(ipmax, ipmax_soc));"
        "else                 % P priority"
        "    ip_cmd = min(max(ip_req, max(-Imax, ipmin_soc)), min(Imax, ipmax_soc));"
        "    iqmax  = sqrt(max(Imax^2 - ip_cmd^2, 0));"
        "    iq_cmd = min(max(iq_req, -iqmax), iqmax);"
        "end"];
    cin = ["ip_req", "iq_req"];
    if isB, cin(end+1) = "SOC"; end
    addMatlabFunction(mdl + "/current limit logic", "current_limit_logic", cin, ...
        ["ip_cmd", "iq_cmd"], body, "par", [900, 100, 1060, 460]);
    L("Pord to Ip/1", "current limit logic/1");
    L("Iq + Iqinj/1", "current limit logic/2");
    if isB, L("SOC/1", "current limit logic/3"); end

    % ---- outports ----
    B("simulink/Sinks/Out1", "ip_cmd",  [1140, 153, 1170, 167], "Port", "1", "OutDataTypeStr", "double");
    B("simulink/Sinks/Out1", "iq_cmd",  [1140, 393, 1170, 407], "Port", "2", "OutDataTypeStr", "double");
    B("simulink/Sinks/Out1", "P_avail", [1140,  13, 1170,  27], "Port", "3", "OutDataTypeStr", "double");
    L("current limit logic/1", "ip_cmd/1");
    L("current limit logic/2", "iq_cmd/1");
    if isB
        L("Pmax/1", "P_avail/1");          % BESS: converter power limit
    else
        L(src + "/1", "P_avail/1");        % PV: available power
    end

    % ---- role areas ----
    addArea(mdl, "Voltage filter and dip detection", [100, 180, 520, 340], areaColor("interface"));
    addArea(mdl, "P path", [130, 0, 700, 175], areaColor("controller"));
    addArea(mdl, "Q path (Qflag 1 closed loop, 0 open loop) and reactive current injection", [350, 360, 850, 700], areaColor("controller"));
    addArea(mdl, "Current limit logic", [880, 50, 1080, 500], areaColor("asset"));

    save_system(mdl, fullfile(folder, mdl + ".slx"));
    close_system(mdl);
    registerInProject(fullfile(folder, mdl + ".slx"));
end

%% ===================================================================
%  WPPC / SPPC / BESSC template: WECC REPC_A plant controller as a block
%  diagram, closing its own loops on the plant feeder into the 33 kV
%  busbar (vendor-style plant controller under the HPPC). P path with
%  frequency droop and PI (Frqflag), Q / V path with PI (RefFlag), then
%  dispatch over the N assets. P and Q references are fed forward around
%  the PI controllers. Ports:
%    in  1 P_set 2 Q_set 3 P_meas 4 Q_meas 5 V_meas 6 P_avail_k [7 SOC_k] last f_meas
%    out 1 P_set_k 2 Q_set_k 3 P_avail 4 P_plant 5 Q_plant
%        [6 P_chg_max 7 P_dis_max 8 SOC]       (pu on the plant rating)
%  ===================================================================
function buildPlantCtrlRepc(name, folder)
    mdl = string(name);
    isB = mdl == "BESSC";
    c = lower(mdl);
    closeIfLoaded(mdl);
    new_system(mdl);
    set_param(mdl, "SolverType", "Fixed-step", "Solver", "FixedStepDiscrete", ...
        "FixedStep", "Ts", "StopTime", "inf", "Description", ...
        mdl + " plant controller: WECC REPC_A structure (P control with frequency droop and PI, Q or " + ...
        "voltage control with PI, references fed forward), closed loop on the plant feeder into the 33 kV " + ...
        "busbar, then dispatch over the N assets by availability. Per unit on the plant rating. " + ...
        "Parameters: model argument par (= p.ctrl." + c + ".par, from p.pc." + c + ").");
    ws = get_param(mdl, "ModelWorkspace");
    ws.DataSource = "MATLAB Code";
    ws.MATLABCode = char(strjoin([
        "% Defaults for standalone use and code generation, from HPP_parameters."
        "% In HPP.slx the Model block passes p.ctrl." + c + ".par as the argument ""par""."
        "pDefault = HPP_parameters();"
        "Ts  = pDefault.ctrl." + c + ".Ts;    % [s] fixed sample time"
        "par = pDefault.ctrl." + c + ".par;   % tunable parameters (model argument)"
        "N   = numel(par.share);   % asset instances (fixed per build)"
        "clear pDefault"], newline));
    reload(ws);
    set_param(mdl, "ParameterArgumentNames", "par");
    B = @(lib, nm, pos, varargin) add_block(lib, mdl + "/" + nm, "Position", pos, varargin{:});
    L = @(a, b) add_line(mdl, a, b, "autorouting", "smart");
    lag = @(nm, T, pos, y0) B("simulink/Discrete/Transfer Fcn First Order", nm, pos, ...
        "PoleZ", "1 - Ts/" + T, "ICPrevOutput", y0);
    pid = @(nm, Kp, Ki, pos) B("simulink/Discrete/Discrete PID Controller", nm, pos, ...
        "Controller", "PI", "P", Kp, "I", Ki, "LimitOutput", "on", "SatLimitsSource", "external", ...
        "AntiWindupMode", "clamping", "SampleTime", "-1");
    K = @(nm, v, pos) B("simulink/Sources/Constant", nm, pos, "Value", v);
    S = @(nm, signs, pos) B("simulink/Math Operations/Sum", nm, pos, "Inputs", signs);
    X = 140:120:2000;   % column x positions

    % ---- inports ----
    ins = ["P_set", "Q_set", "P_meas", "Q_meas", "V_meas", "P_avail_k"];
    yIn = [120, 470, 300, 560, 660, 820];
    dimsIn = ["1", "1", "1", "1", "1", "N"];
    if isB
        ins(end+1) = "SOC_k"; yIn(end+1) = 900; dimsIn(end+1) = "N";
    end
    ins(end+1) = "f_meas"; yIn(end+1) = 200; dimsIn(end+1) = "1";
    for k = 1:numel(ins)
        B("simulink/Sources/In1", ins(k), [30, yIn(k)-7, 60, yIn(k)+7], "Port", num2str(k), ...
            "OutDataTypeStr", "double", "PortDimensions", dimsIn(k), "SampleTime", "Ts");
    end

    % ---- availability and P limits ----
    if isB
        body = [
            "% plant charge / discharge limits: only assets within their SOC range"
            "dis = P_avail_k .* (SOC_k > par.SOCmin);"
            "chg = P_avail_k .* (SOC_k < par.SOCmax);"
            "P_dis_max = sum(dis);"
            "P_chg_max = sum(chg);"
            "SOC = sum(par.share .* SOC_k);"];
        addMatlabFunction(mdl + "/BESS limits", "bess_limits", ["P_avail_k", "SOC_k"], ...
            ["P_chg_max", "P_dis_max", "SOC"], body, "par", [X(2), 800, X(2)+130, 920]);
        L("P_avail_k/1", "BESS limits/1");  L("SOC_k/1", "BESS limits/2");
        B("simulink/Math Operations/Gain", "charge limit", [X(3), 790, X(3)+50, 820], "Gain", "-1");
        L("BESS limits/1", "charge limit/1");
        upSrc = "BESS limits/2";  loSrc = "charge limit/1";
    else
        K("Pmin", "0", [X(3), 860, X(3)+40, 880]);
        upSrc = "P_avail sum/1";  loSrc = "Pmin/1";
    end
    B("simulink/Math Operations/Sum of Elements", "P_avail sum", [X(2), 740, X(2)+40, 770]);
    L("P_avail_k/1", "P_avail sum/1");

    % ---- P path: frequency droop, PI on P (Frqflag), Tlag ----
    K("Freq_ref", "par.Freq_ref", [X(1), 165, X(1)+50, 185]);
    S("f error", "+-", [X(2), 175, X(2)+30, 215]);
    L("Freq_ref/1", "f error/1");  L("f_meas/1", "f error/2");
    B("simulink/Discontinuities/Dead Zone", "fdbd", [X(3), 180, X(3)+50, 210], ...
        "LowerValue", "par.fdbd1", "UpperValue", "par.fdbd2");
    L("f error/1", "fdbd/1");
    B("simulink/Discontinuities/Saturation", "under-f", [X(4), 160, X(4)+40, 190], "UpperLimit", "inf", "LowerLimit", "0");
    B("simulink/Discontinuities/Saturation", "over-f", [X(4), 210, X(4)+40, 240], "UpperLimit", "0", "LowerLimit", "-inf");
    L("fdbd/1", "under-f/1");  L("fdbd/1", "over-f/1");
    B("simulink/Math Operations/Gain", "Dup", [X(5), 160, X(5)+40, 190], "Gain", "par.Dup");
    B("simulink/Math Operations/Gain", "Ddn", [X(5), 210, X(5)+40, 240], "Gain", "par.Ddn");
    L("under-f/1", "Dup/1");  L("over-f/1", "Ddn/1");
    S("P_set + droop", "+++", [X(6), 100, X(6)+30, 240]);
    L("P_set/1", "P_set + droop/1");  L("Dup/1", "P_set + droop/2");  L("Ddn/1", "P_set + droop/3");
    lag("P filt", "par.Tp", [X(1), 283, X(1)+80, 317], "0");  L("P_meas/1", "P filt/1");
    S("P error", "+-", [X(7), 250, X(7)+30, 310]);
    L("P_set + droop/1", "P error/1");  L("P filt/1", "P error/2");
    B("simulink/Discontinuities/Saturation", "femax femin", [X(8), 265, X(8)+40, 295], ...
        "UpperLimit", "par.femax", "LowerLimit", "par.femin");
    L("P error/1", "femax femin/1");
    S("Pmax - ref", "+-", [X(8), 330, X(8)+30, 370]);
    S("Pmin - ref", "+-", [X(8), 390, X(8)+30, 430]);
    L(upSrc, "Pmax - ref/1");  L("P_set + droop/1", "Pmax - ref/2");
    L(loSrc, "Pmin - ref/1");  L("P_set + droop/1", "Pmin - ref/2");
    pid("Kpg Kig", "par.Kpg", "par.Kig", [X(9), 260, X(9)+80, 380]);
    L("femax femin/1", "Kpg Kig/1");  L("Pmax - ref/1", "Kpg Kig/2");  L("Pmin - ref/1", "Kpg Kig/3");
    S("ref + PI", "++", [X(10), 160, X(10)+30, 340]);
    L("P_set + droop/1", "ref + PI/1");  L("Kpg Kig/1", "ref + PI/2");
    K("Frqflag", "par.Frqflag", [X(10), 60, X(10)+50, 80]);
    B("simulink/Signal Routing/Switch", "closed loop?", [X(11), 100, X(11)+40, 180], "Criteria", "u2 ~= 0");
    L("ref + PI/1", "closed loop?/1");  L("Frqflag/1", "closed loop?/2");  L("P_set/1", "closed loop?/3");
    B("simulink/Discontinuities/Saturation Dynamic", "Pmax Pmin", [X(12), 100, X(12)+50, 200]);
    L(upSrc, "Pmax Pmin/1");  L("closed loop?/1", "Pmax Pmin/2");  L(loSrc, "Pmax Pmin/3");
    lag("Tlag", "par.Tlag", [X(13), 133, X(13)+80, 167], "0");
    L("Pmax Pmin/1", "Tlag/1");

    % ---- Q / V path: RefFlag, deadband, error limits, freeze, PI, Tfv ----
    lag("Q filt", "par.Tfltr", [X(1), 543, X(1)+80, 577], "0");  L("Q_meas/1", "Q filt/1");
    lag("V filt", "par.Tfltr", [X(1), 643, X(1)+80, 677], "1");  L("V_meas/1", "V filt/1");
    B("simulink/Math Operations/Gain", "Kc", [X(2), 590, X(2)+40, 620], "Gain", "par.Kc");
    L("Q filt/1", "Kc/1");
    S("V + Kc Q", "++", [X(3), 620, X(3)+30, 670]);
    L("V filt/1", "V + Kc Q/1");  L("Kc/1", "V + Kc Q/2");
    K("Vref", "par.Vref", [X(3), 700, X(3)+40, 720]);
    S("V error", "+-", [X(4), 640, X(4)+30, 710]);
    L("Vref/1", "V error/1");  L("V + Kc Q/1", "V error/2");
    S("Q error", "+-", [X(4), 470, X(4)+30, 560]);
    L("Q_set/1", "Q error/1");  L("Q filt/1", "Q error/2");
    K("RefFlag", "par.RefFlag", [X(4), 590, X(4)+50, 610]);
    B("simulink/Signal Routing/Switch", "V control?", [X(5), 540, X(5)+40, 660], "Criteria", "u2 ~= 0");
    L("V error/1", "V control?/1");  L("RefFlag/1", "V control?/2");  L("Q error/1", "V control?/3");
    B("simulink/Discontinuities/Dead Zone", "dbd", [X(6), 585, X(6)+50, 615], ...
        "LowerValue", "par.dbd1", "UpperValue", "par.dbd2");
    L("V control?/1", "dbd/1");
    B("simulink/Discontinuities/Saturation", "emax emin", [X(7), 585, X(7)+40, 615], ...
        "UpperLimit", "par.emax", "LowerLimit", "par.emin");
    L("dbd/1", "emax emin/1");
    K("Vfrz", "par.Vfrz", [X(6), 720, X(6)+40, 740]);
    B("simulink/Logic and Bit Operations/Relational Operator", "V < Vfrz", [X(7), 680, X(7)+40, 730], "Operator", "<");
    L("V filt/1", "V < Vfrz/1");  L("Vfrz/1", "V < Vfrz/2");
    K("frozen", "0", [X(7), 640, X(7)+40, 660]);
    B("simulink/Signal Routing/Switch", "freeze", [X(8), 600, X(8)+40, 700], "Criteria", "u2 ~= 0");
    L("frozen/1", "freeze/1");  L("V < Vfrz/1", "freeze/2");  L("emax emin/1", "freeze/3");
    % feed-forward: Q_set in Q control, none in voltage control
    K("no ff", "0", [X(5), 420, X(5)+40, 440]);
    B("simulink/Signal Routing/Switch", "Q feed-forward", [X(6), 440, X(6)+40, 520], "Criteria", "u2 ~= 0");
    L("no ff/1", "Q feed-forward/1");  L("RefFlag/1", "Q feed-forward/2");  L("Q_set/1", "Q feed-forward/3");
    K("Qmax", "par.Qmax", [X(7), 460, X(7)+40, 480]);
    K("Qmin", "par.Qmin", [X(7), 510, X(7)+40, 530]);
    S("Qmax - ff", "+-", [X(8), 450, X(8)+30, 490]);
    S("Qmin - ff", "+-", [X(8), 510, X(8)+30, 550]);
    L("Qmax/1", "Qmax - ff/1");  L("Q feed-forward/1", "Qmax - ff/2");
    L("Qmin/1", "Qmin - ff/1");  L("Q feed-forward/1", "Qmin - ff/2");
    pid("Kp Ki", "par.Kp", "par.Ki", [X(9), 500, X(9)+80, 620]);
    L("freeze/1", "Kp Ki/1");  L("Qmax - ff/1", "Kp Ki/2");  L("Qmin - ff/1", "Kp Ki/3");
    S("ff + PI", "++", [X(10), 470, X(10)+30, 560]);
    L("Q feed-forward/1", "ff + PI/1");  L("Kp Ki/1", "ff + PI/2");
    lag("Tfv", "par.Tfv", [X(11), 498, X(11)+80, 532], "0");
    L("ff + PI/1", "Tfv/1");

    % ---- dispatch over the assets ----
    body = [
        "% Split the plant P and Q orders over the assets in proportion to"
        "% their availability (equal shares if nothing is available)."];
    if isB
        body = [body
            "% BESS: only assets that may discharge (charge) share a discharge (charge) order."
            "if P_ord >= 0"
            "    a = P_avail_k .* (SOC_k > par.SOCmin);"
            "else"
            "    a = P_avail_k .* (SOC_k < par.SOCmax);"
            "end"];
        din = ["P_ord", "Q_ord", "P_avail_k", "SOC_k"];
    else
        body = [body; "a = P_avail_k;"];
        din = ["P_ord", "Q_ord", "P_avail_k"];
    end
    body = [body
        "s = sum(a);"
        "if s > 1e-6"
        "    w = a / s;"
        "else"
        "    w = par.share;"
        "end"
        "P_set_k = P_ord * w;"
        "Q_set_k = Q_ord * par.share;   % Q by rating share (also with no availability)"];
    addMatlabFunction(mdl + "/dispatch", "dispatch", din, ["P_set_k", "Q_set_k"], body, "par", ...
        [X(14), 120, X(14)+130, 540]);
    L("Tlag/1", "dispatch/1");  L("Tfv/1", "dispatch/2");  L("P_avail_k/1", "dispatch/3");
    if isB, L("SOC_k/1", "dispatch/4"); end

    % ---- outports ----
    outs = ["P_set_k", "Q_set_k", "P_avail", "P_plant", "Q_plant"];
    yo = [200, 460, 760, 300, 560];
    srcs = ["dispatch/1", "dispatch/2", "P_avail sum/1", "P_meas/1", "Q_meas/1"];
    if isB
        outs = [outs, "P_chg_max", "P_dis_max", "SOC"];  yo = [yo, 820, 860, 900];
        srcs = [srcs, "BESS limits/1", "BESS limits/2", "BESS limits/3"];
    end
    xo = X(14) + 220;
    for k = 1:numel(outs)
        B("simulink/Sinks/Out1", outs(k), [xo, yo(k)-7, xo+30, yo(k)+7], "Port", num2str(k), "OutDataTypeStr", "double");
        L(srcs(k), outs(k) + "/1");
    end

    % ---- role areas ----
    addArea(mdl, "Frequency response (Ddn, Dup)", [X(1)-10, 150, X(5)+60, 250], areaColor("interface"));
    addArea(mdl, "P control (Frqflag)", [X(6)-10, 40, X(13)+100, 440], areaColor("controller"));
    addArea(mdl, "Q or V control (RefFlag)", [X(2)-10, 405, X(11)+100, 745], areaColor("controller"));
    addArea(mdl, "Dispatch to the assets", [X(14)-10, 100, X(14)+150, 560], areaColor("asset"));

    save_system(mdl, fullfile(folder, mdl + ".slx"));
    close_system(mdl);
    registerInProject(fullfile(folder, mdl + ".slx"));
end
