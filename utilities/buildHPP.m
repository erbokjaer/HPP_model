function buildHPP(opts)
%BUILDHPP  (Re)build the hierarchical HPP model from the parameters.
%
%   buildHPP()                    rebuild the top model HPP.slx for the
%                                 asset counts N in HPP_parameters()
%   buildHPP(All=true)            also create controller and asset models
%                                 that do not exist yet, from the templates;
%                                 existing ones are never touched
%   buildHPP(All=true, Overwrite=true, Only="HPPC")  regenerate the named
%                                 models from their templates: OVERWRITES
%                                 them, including all hand edits (controller
%                                 names or "<type>_asset"; without Only: all)
%   buildHPP(..., Strict=true)    stop if checkModels finds a problem in the
%                                 controller / asset models (default: warn)
%   Force=true (older option) only works together with Overwrite=true.
%
%   Ownership: HPP.slx is always generated (its structure depends on N), so
%   never edit it by hand. The controller and asset models (models/ctrl,
%   models/assets) are the source: develop them in the Simulink editor. The
%   templates in this file only create models that are missing. Keep the
%   port names and order (modelInterfaces.m; HPP.slx connects by port), and
%   in block parameters that use par / ap only + - * / and field access
%   (compute anything else in params/controllerParameters.m). checkModels
%   checks both and runs at the start of buildHPP and of Rapid Accelerator
%   builds.
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
%
%   The Network block holds both network models as variants (nodal solver
%   and Simscape Electrical); the active one is chosen per run, see runCase
%   (Network option) and p.net.solverType.

    arguments
        opts.All (1,1) logical = false
        opts.Force (1,1) logical = false       % older option: needs Overwrite=true
        opts.Overwrite (1,1) logical = false   % regenerate existing controller / asset models
        opts.Only (1,:) string = strings(1, 0) % with All: create / regenerate only these models
        opts.Strict (1,1) logical = false      % error on checkModels findings
    end
    if opts.Force && ~opts.Overwrite
        error("buildHPP:overwrite", "Force=true would regenerate the controller and asset models from the " + ...
            "templates and overwrite all hand edits. The models are the source now; to really overwrite, " + ...
            "say so: buildHPP(All=true, Overwrite=true, Only=[...]).");
    end
    if opts.Overwrite && ~opts.All
        error("buildHPP:overwriteAll", "Overwrite=true applies to the controller and asset models; use it " + ...
            "with All=true, e.g. buildHPP(All=true, Overwrite=true, Only=""HPPC"").");
    end

    % hand edits in the controller / asset models (expressions, ports, fields)
    findings = checkModels(Quiet=true);
    if ~isempty(findings)
        checkModels(Strict=opts.Strict);                % prints the report (and errors with Strict)
        warning("buildHPP:checkModels", "checkModels found %d problem(s) in the controller / asset models " + ...
            "(listed above). Rapid Accelerator will fall back to normal mode until they are fixed.", height(findings));
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
            if ~isempty(opts.Only) && ~any(opts.Only == specs(k).name)
                continue
            end
            if opts.Overwrite || ~isfile(fullfile(dirs.ctrl, specs(k).name + ".slx"))
                if specs(k).name == "HPPC"
                    buildHppcCtrl(dirs.ctrl);           % stage 5: closed-loop P, frequency response
                elseif specs(k).name == "WT_ctrl"
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
            if ~isempty(opts.Only) && ~any(opts.Only == type + "_asset")
                continue
            end
            if opts.Overwrite || ~isfile(fullfile(dirs.assets, type + "_asset.slx"))
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
          "P_avail_wpp","P_wpp","Q_wpp","Q_sat_wpp", ...
          "P_avail_spp","P_spp","Q_spp","Q_sat_spp", ...
          "P_chg_max_bess","P_dis_max_bess","SOC_bess","P_bess","Q_bess","Q_sat_bess"];
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
        "HPP controller. Per unit on p.ctrl.hppc.Pbase (POC base). Model: buildHppcCtrl (stage 5).");

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
        out = [out, dims("Q_sat")]; %#ok<AGROW>   % Q order at its limit (HPPC anti-windup)
        body = [body; "Q_sat = 0;"]; %#ok<AGROW>
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
    set_param(m, "SolverType", "Fixed-step", "Solver", "ode1", "FixedStep", "p.sim.Ts", ...
        "SolverMode", "SingleTasking", "Decimation", "p.sim.logDecimation", ...
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
    buildNetworkVariants(m + "/Network", p);       % nodal and Simscape, chosen per run
    buildGridFrequency(m + "/Grid frequency");     % imposed or swing equation
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
    yN = get_param(m + "/Network", "Position");
    setPos(m + "/Grid frequency", [990, yN(4) + 120, 1190, yN(4) + 200]);
    setPos(m + "/Logging", [1510 60 1630 60 + numel(logSignalNames())*36]);

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
    gotoFrom(m, "Wind plant/1", ["HPP controller/3", "Logging/6"], "meas_wpp", orange, "meas wpp");
    gotoFrom(m, "PV plant/1", ["HPP controller/4", "Logging/7"], "meas_spp", orange, "meas spp");
    gotoFrom(m, "TSO/1", "Logging/5", "tso", grey, "TSO");
    gotoFrom(m, "BESS plant/1", ["HPP controller/5", "Logging/4"], "meas_bess", orange, "meas bess");
    gotoFrom(m, "Network/1", ["Wind plant/2", "PV plant/2", "BESS plant/2", "Logging/1"], "network", green, "network");
    gotoFrom(m, "Network/2", ["HPP controller/2", "Logging/2", "Grid frequency/1"], "POC_meas", green, "POC meas");
    gotoFrom(m, "Grid frequency/1", ["TSO/1", "Network/4"], "f_grid", green, "f");
    gotoFrom(m, "Grid frequency/2", "Network/5", "theta_grid", green, "theta");
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

    logNames = logSignalNames();
    lp = get_param(m + "/Logging", "PortConnectivity");
    nIn = numel(get_param(m + "/Logging", "PortHandles").Inport);
    for k = 1:numel(logNames)
        y = lp(nIn + k).Position(2);      % align each outport with its Logging port
        add_block("simulink/Sinks/Out1", m + "/" + logNames(k), "Port", num2str(k), ...
            "Position", [1710 y-7 1740 y+7]);
        ln(m, sprintf("Logging/%d", k), logNames(k) + "/1", logNames(k));
    end

    % record what this build was made for; runCase/runSweep check it
    ws = get_param(m, "ModelWorkspace");
    assignin(ws, "buildInfo", struct("N_wt", p.wt.N, "N_pv", p.pv.N, "N_bess", p.bess.N, ...
        "built", string(datetime("now"))));
    % active network variant for runs started from the editor (runCase sets it per run)
    assignin(ws, "HPP_network", networkCode(p.net.solverType));
    [~, dc] = delayMode(p, table());
    assignin(ws, "HPP_delays", dc);
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
        if names(j) == "f_meas"   % grid frequency from the Grid frequency block
            add_block("simulink/Sources/In1", sys + "/" + names(j), "Port", "1", "Position", [0 0 30 14]);
        elseif names(j) == "P_ref"   % power reference with the optional step (p.grid.Pstep)
            add_block("simulink/Sources/Step", sys + "/" + names(j), "Time", "p.grid.Pstep.t0", ...
                "Before", "p.grid.Pref", "After", "p.grid.Pref + p.grid.Pstep.dP", "SampleTime", "0", ...
                "Position", [0 0 90 26]);
        elseif names(j) == "Q_ref"   % reactive power reference with the optional step (p.grid.Qstep)
            add_block("simulink/Sources/Step", sys + "/" + names(j), "Time", "p.grid.Qstep.t0", ...
                "Before", "p.grid.Qref", "After", "p.grid.Qref + p.grid.Qstep.dQ", "SampleTime", "0", ...
                "Position", [0 0 90 26]);
        else
            add_block("simulink/Sources/Constant", sys + "/" + names(j), "Value", vals(j), "Position", [0 0 90 26]);
        end
        rowInto(sys, "TSO bus/" + j, names(j), names(j), 50);
    end
    add_block("simulink/Sinks/Out1", sys + "/TSO", "Port", "1", "Position", [0 0 30 14]);
    rowFrom(sys, "TSO bus/1", "TSO", "", 50);
    set_param(sys, "Description", "TSO / market set-points: P_ref [W], Q_ref [var]; f_meas [pu] passed on from the Grid frequency block.");
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
    % into the busbar, V_bus [pu], grid frequency f [pu]); 2 POC bus
    % (P_poc [W], Q_poc [var], V_poc [pu]), power into the grid.
    % Layout as the controllers (rowChain): injections and grid source |
    % network solution | output buses, signals passed with Goto/From.
    newSubsystem(sys);
    cm.ref  = ["I_re_wpp", "I_im_wpp", "I_re_spp", "I_im_spp", "I_re_bess", "I_im_bess", "I_re", "I_im", "E"];
    cm.meas = ["V_re", "V_im", "V_re_prev", "V_im_prev", "V_bus", "V_poc", "P_plant", "Q_plant", ...
               "P_poc", "Q_poc", "P_plant_pu", "Q_plant_pu", "P_poc_pu", "Q_poc_pu"];

    % ---- current injections (system pu) and grid source voltage ----
    tags = ["wpp", "spp", "bess"];
    for i = 1:3
        add_block("simulink/Sources/In1", sys + "/I " + tags(i), "Port", num2str(i), "Position", [0 0 30 14]);
        add_block("simulink/Signal Routing/Bus Selector", sys + "/I sel " + tags(i), ...
            "Position", [0 0 6 60], "ShowName", "off");
    end
    add_block("simulink/Signal Routing/Mux", sys + "/I_re all", "Inputs", "3", "Position", [0 0 6 90]);
    add_block("simulink/Signal Routing/Mux", sys + "/I_im all", "Inputs", "3", "Position", [0 0 6 90]);
    add_block("simulink/Sources/Constant", sys + "/E0", "Value", "net.E", "Position", [0 0 50 24]);
    add_block("simulink/Sources/Step", sys + "/dip start", "Time", "p.grid.dip.t0", ...
        "Before", "0", "After", "p.grid.dip.depth", "SampleTime", "0", "Position", [0 0 30 30]);
    add_block("simulink/Sources/Step", sys + "/dip end", "Time", "p.grid.dip.t0 + p.grid.dip.dur", ...
        "Before", "0", "After", "p.grid.dip.depth", "SampleTime", "0", "Position", [0 0 30 30]);
    nb(sys, "sum", "E grid", "Inputs", "+-+");
    set_param(sys + "/E grid", "Position", [0 0 30 170]);
    rows = cell(0, 1);
    for i = 1:3
        rows{end+1, 1} = [stg("I " + tags(i), 0), stg("I sel " + tags(i), 1, {}, {2, "I_im_" + tags(i)}), ...
                          stg(">I_re_" + tags(i))]; %#ok<AGROW>
    end
    rows = [rows
        {[stg("@I_re_wpp", 0), stg("I_re all", 1, {2, "@I_re_spp"; 3, "@I_re_bess"}), stg(">I_re")]}
        {[stg("@I_im_wpp", 0), stg("I_im all", 1, {2, "@I_im_spp"; 3, "@I_im_bess"}), stg(">I_im")]}
        {[stg("E0", 0), stg("E grid", 1, {2, "dip start"; 3, "dip end"}), stg(">E")]}];
    % bus selectors need their elements before rowChain sizes the rows
    for i = 1:3
        set_param(sys + "/I sel " + tags(i), "OutputSignals", "I_re,I_im");
    end
    [~, hIn, rIn] = ctrlSection(sys, "Current injections and grid source (dip)", rows, 40, 20, cm, areaColor("interface"));

    % ---- network solution ----
    outs = ["V_re", "V_im", "P_plant", "Q_plant", "V_bus", "P_poc", "Q_poc", "V_poc"];
    body = [
        "% numeric network data come in as (tunable) inputs; net holds the structure"
        "Z  = complex(Zre, Zim);"
        "nA = numel(I_re);"
        "n  = size(Z, 1);"
        "Zg = complex(Zg_ri(1), Zg_ri(2));"
        "I = complex(zeros(n, 1), zeros(n, 1));"
        "I(1:nA) = complex(I_re(:), I_im(:));"
        "Ec = E * exp(1i * theta);                 % grid source, rotated by the grid angle (Grid frequency)"
        "I(net.poc) = I(net.poc) + Ec / Zg;        % grid Norton source at the POC (E: grid voltage, incl. dips)"
        "V = Z * I;"
        "V_re = real(V(1:nA));"
        "V_im = imag(V(1:nA));"
        "% power each plant delivers into the HPP busbar (its measurement point):"
        "% flows of its plant branches at the busbar end, incl. bus-end charging"
        "Vb = V(net.bus);"
        "Ibr = (V(net.brFrom) - Vb) ./ complex(brZre, brZim) - 1i * brB .* Vb;"
        "Sbr = Vb .* conj(Ibr);"
        "P_plant = zeros(3, 1);  Q_plant = zeros(3, 1);"
        "for j = 1:numel(Sbr)"
        "    P_plant(net.brPlant(j)) = P_plant(net.brPlant(j)) + real(Sbr(j));"
        "    Q_plant(net.brPlant(j)) = Q_plant(net.brPlant(j)) + imag(Sbr(j));"
        "end"
        "V_bus = abs(Vb);"
        "Vp = V(net.poc);"
        "Spoc = Vp * conj((Vp - Ec) / Zg);         % power into the grid"
        "P_poc = real(Spoc);  Q_poc = imag(Spoc);  V_poc = abs(Vp);"];
    ins = ["I_re", "I_im", "E", "Zre", "Zim", "Zg_ri", "brZre", "brZim", "brB", "theta"];
    add_block("simulink/Sources/In1", sys + "/theta", "Port", "5", "Position", [0 0 30 14]);
    addMatlabFunction(sys + "/solve", "solve", ins, outs, body, "net", [0 0 160 360]);
    % the structure (node indices) stays fixed at compile time; the numbers
    % (impedances, charging, grid) are tunable constants, so grid strength,
    % cable and transformer data change without recompiling
    d = sfroot().find("-isa", "Stateflow.EMChart", "Path", char(sys + "/solve")).find("-isa", "Stateflow.Data", "Name", "net");
    d.Tunable = false;
    vals = ["net.Zre", "net.Zim", "[net.Zg_re, net.Zg_im]", "net.brZre", "net.brZim", "net.brB"];
    for j = 1:6
        kc(sys, ins(3 + j), vals(j));
    end
    outTags = ["V_re", "V_im", "P_plant_pu", "Q_plant_pu", "V_bus", "P_poc_pu", "Q_poc_pu", "V_poc"];
    outsS = [num2cell((2:8)'), num2cell(outTags(2:8)')];
    sidesS = [{2, "@I_im"; 3, "@E"}; num2cell((4:9)'), num2cell(ins(4:9)'); {10, "theta"}];
    rowsS = {[stg("@I_re", 0), stg("solve", 1, sidesS, outsS), stg(">" + outTags(1))]};
    [~, hS, rS] = ctrlSection(sys, "Network solution, V = Z (I_inj + I_grid)", rowsS, rIn + 90, 20, cm, areaColor("asset"));

    % ---- output buses: terminal voltages one step old (generator "PLL";
    %      avoids an algebraic loop), powers from system pu to W / var ----
    add_block("simulink/Discrete/Memory", sys + "/V_re prev", "InitialCondition", "ones(net.nA,1)", "Position", [0 0 30 30]);
    add_block("simulink/Discrete/Memory", sys + "/V_im prev", "InitialCondition", "zeros(net.nA,1)", "Position", [0 0 30 30]);
    for s = ["P_plant", "Q_plant", "P_poc", "Q_poc"]
        nb(sys, "gain", s + " to SI", "Gain", "net.Sb");
    end
    add_block("simulink/Sources/In1", sys + "/f grid", "Port", "4", "Position", [0 0 30 14]);
    add_block("simulink/Signal Routing/Bus Creator", sys + "/network bus", "Inputs", "6", "Position", [0 0 6 300]);
    add_block("simulink/Signal Routing/Bus Creator", sys + "/POC bus", "Inputs", "3", "Position", [0 0 6 150]);
    add_block("simulink/Sinks/Out1", sys + "/network", "Port", "1", "Position", [0 0 30 14]);
    add_block("simulink/Sinks/Out1", sys + "/POC meas", "Port", "2", "Position", [0 0 30 14]);
    rowsO = {
        [stg("@V_im", 0), stg("V_im prev"), stg(">V_im_prev")]
        [stg("@P_plant_pu", 0), stg("P_plant to SI"), stg(">P_plant")]
        [stg("@Q_plant_pu", 0), stg("Q_plant to SI"), stg(">Q_plant")]
        [stg("@V_re", 0), stg("V_re prev"), stg("network bus", 1, {2, "@V_im_prev"; 3, "@P_plant"; 4, "@Q_plant"; ...
         5, "@V_bus"; 6, "f grid"}), stg("network")]
        [stg("@P_poc_pu", 0), stg("P_poc to SI"), stg(">P_poc")]
        [stg("@Q_poc_pu", 0), stg("Q_poc to SI"), stg(">Q_poc")]
        [stg("@P_poc", 0), stg("POC bus", 1, {2, "@Q_poc"; 3, "@V_poc"}), stg("POC meas")]};
    [~, hO] = ctrlSection(sys, "Output buses (V one step old, powers in W and var)", rowsO, rS + 90, 20, cm, areaColor("interface"));
    % bus element names come from the names of the lines into the creators
    nameInputs(sys + "/network bus", ["V_re", "V_im", "P_plant", "Q_plant", "V_bus", "f"]);
    nameInputs(sys + "/POC bus", ["P_poc", "Q_poc", "V_poc"]);
    alignColumns(hIn, hS, hO);

    mk = Simulink.Mask.create(sys);
    mk.addParameter("Name", "net", "Prompt", "Network solver data", "Value", "p.net.solver", ...
        "Evaluate", "on", "Tunable", "on");   % tunable: the numeric network data (constants) change without recompiling
    mk.Description = "Positive-sequence phasor network (collector system, transformers, grid). " + ...
        "Solves V = Z*I each step. Data from networkSolverData(p).";
end

function nameInputs(blk, names)
    ph = get_param(blk, "PortHandles");
    for k = 1:numel(names)
        set_param(get_param(ph.Inport(k), "Line"), "Name", names(k));
    end
end

% ---------------------------------------------------------------------
function buildLogging(sys)
    % Picks the logged signals out of the buses; its outputs feed the
    % top-level outports (out.yout). One straight row per logged signal.
    newSubsystem(sys);
    order = logSignalNames();
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
    y = y + 3*(3*pitch+30) + 20;

    % TSO references and grid frequency; available power of wind and PV
    groups = {"TSO", 5, ["P_ref", "Q_ref", "f_meas"], ["P_ref", "Q_ref", "f"]; ...
              "meas wpp", 6, "P_avail", "P_avail_wpp"; ...
              "meas spp", 7, "P_avail", "P_avail_spp"};
    for g = 1:size(groups, 1)
        [gname, gport, elems, logn] = groups{g, :};
        inport(sys, gname, gport);
        sel = gname + " sel";
        add_block("simulink/Signal Routing/Bus Selector", sys + "/" + sel, ...
            "Position", [xs y xs+5 y+numel(elems)*pitch]);
        rowInto(sys, sel + "/1", gname, "", 50);
        set_param(sys + "/" + sel, "OutputSignals", char(strjoin(elems, ",")));
        for j = 1:numel(elems)
            add_block("simulink/Sinks/Out1", sys + "/" + logn(j), "Port", port(logn(j)), "Position", [0 0 30 14]);
            rowFrom(sys, sel + "/" + j, logn(j), "", 200);
        end
        y = y + numel(elems)*pitch + 20;
    end
    % ports were created in row order; set the final numbers explicitly
    renumberPorts(sys, order);
    renumberPorts(sys, ["network", "POC meas", "set-points bess", "meas bess", "TSO", "meas wpp", "meas spp"]);
    set_param(sys, "Description", "Selects the logged signals (W, var, pu) for the top-level outports.");
    hideStructuralNames(sys);
end

function names = logSignalNames()
    % logged signals in outport order (out.yout); plotAll reads them by name
    names = ["P_poc","SOC","P_bess_set","Q_poc","P_wpp","P_spp","P_bess","Q_wpp","Q_spp","Q_bess", ...
             "V_poc","V_bus","P_ref","Q_ref","f","P_avail_wpp","P_avail_spp"];
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
%  missing unless Force=true). Same layout as the controllers (see
%  rowChain): inputs | terminal measurements, asset controller, generator
%  / converter, (battery energy) | outputs. Ports:
%    in  1 P_set 2 Q_set 3 V_re 4 V_im [5 P_wind_avail | P_pv_avail]
%    out 1 P 2 Q 3 P_avail 4 I_re 5 I_im [6 SOC]
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
    isB = type == "BESS";
    switch type
        case "WT", src = "P_wind_avail";
        case "PV", src = "P_pv_avail";
        otherwise, src = "";
    end
    cm.ref  = ["P_set", "Q_set", "ip_cmd", "iq_cmd", "ip", "iq"];
    cm.meas = ["V_re", "V_im", "V_term", "P_meas", "Q_meas", "P", "Q", "SOC", "P_avail", ...
               "P_wind_avail", "P_pv_avail", "I_re", "I_im"];
    ins = ["P_set", "Q_set", "V_re", "V_im"];
    if src ~= "", ins(end+1) = src; end
    [hIn, x] = ctrlInputs(mdl, ins, repmat("1", 1, numel(ins)), cm, "-1");

    % terminal measurements: |V|, P and Q one step old
    addMatlabFunction(mdl + "/terminal", "terminal", ["V_re", "V_im"], "V_term", ...
        "V_term = sqrt(V_re^2 + V_im^2);", "", [0 0 90 60]);
    nb(mdl, "mem", "P meas");  nb(mdl, "mem", "Q meas");
    rowsM = {
        [stg("@V_re", 0), stg("terminal", 1, {2, "@V_im"}), stg(">V_term")]
        [stg("@P", 0), stg("P meas"), stg(">P_meas")]
        [stg("@Q", 0), stg("Q meas"), stg(">Q_meas")]};
    [y, h1, r1] = ctrlSection(mdl, "Terminal measurements", rowsM, x, 20, cm, areaColor("interface"));

    % asset controller (future DLL), ports 1 P_set 2 Q_set 3 V_term 4 P_meas 5 Q_meas 6 source
    cb = mdl + "/ctrl";
    add_block("simulink/Ports & Subsystems/Model", cb, "ModelName", ctrl, "Position", [0 0 180 210]);
    Simulink.ModelReference.refresh(cb);
    setArg(cb, "par", "ap.ctrl");
    set_param(cb, "BackgroundColor", areaColor("controller"));
    if src ~= "", s6 = "@" + src; else, s6 = "@SOC"; end
    rowsC = {[stg("@P_set", 0), stg("ctrl", 1, {2, "@Q_set"; 3, "@V_term"; 4, "@P_meas"; 5, "@Q_meas"; 6, s6}, ...
              {2, "iq_cmd"; 3, "P_avail"}), stg(">ip_cmd")]};
    [y, h2, r2] = ctrlSection(mdl, "Asset controller (" + ctrl + ")", rowsC, x, y, cm, areaColor("controller"));

    % generator / converter: current dynamics, then a current source
    % aligned with the terminal voltage, I = (ip - j*iq) * V/|V|,
    % P = |V|*ip, Q = |V|*iq (+ = export / capacitive)
    gen = [
        "v  = max(sqrt(V_re^2 + V_im^2), 1e-3);"
        "cr = V_re / v;  ci = V_im / v;"
        "I_re = ip*cr + iq*ci;"
        "I_im = ip*ci - iq*cr;"
        "P = v * ip;"
        "Q = v * iq;"];
    addMatlabFunction(mdl + "/generator", "generator", ["ip", "iq", "V_re", "V_im"], ...
        ["P", "Q", "I_re", "I_im"], gen, "", [0 0 120 150]);
    rowGen = [stg("@ip", 0), stg("generator", 1, {2, "@iq"; 3, "@V_re"; 4, "@V_im"}, ...
              {2, "Q"; 3, "I_re"; 4, "I_im"}), stg(">P")];
    if type == "WT"
        % IEC 61400-27-1 Type 4 generator system: current rate limits, then
        % a first-order lag Tg (active current: only its rise is limited)
        nb(mdl, "rate", "dipmax", "RisingSlewLimit", "ap.gen.dipmax", "FallingSlewLimit", "-1e6");
        contLag(mdl, "Tg ip", "ap.gen.Tg");
        nb(mdl, "rate", "diqmax", "RisingSlewLimit", "ap.gen.diqmax", "FallingSlewLimit", "-ap.gen.diqmax");
        contLag(mdl, "Tg iq", "ap.gen.Tg");
        rowsG = {
            [stg("@ip_cmd", 0), stg("dipmax"), stg("Tg ip"), stg(">ip")]
            [stg("@iq_cmd", 0), stg("diqmax"), stg("Tg iq"), stg(">iq")]
            rowGen};
        gTitle = "Generator system (IEC 61400-27-1 Type 4)";
    else
        % WECC REGC_A: Ip recovery ramp Rrpwr, Iq ramps Iqrmax/Iqrmin, lag Tg;
        % low voltage active current management (LVACM) and low voltage
        % power logic (LVPL, static, on the unfiltered voltage)
        nb(mdl, "rate", "Rrpwr", "RisingSlewLimit", "ap.gen.Rrpwr", "FallingSlewLimit", "-1e6");
        contLag(mdl, "Tg ip", "ap.gen.Tg");
        nb(mdl, "rate", "Iqrmax", "RisingSlewLimit", "ap.gen.Iqrmax", "FallingSlewLimit", "ap.gen.Iqrmin");
        contLag(mdl, "Tg iq", "ap.gen.Tg");
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
        addMatlabFunction(mdl + "/LVPL LVACM", "lvpl_lvacm", ["ip", "V_re", "V_im"], "ip_out", lv, "ap", [0 0 110 100]);
        rowsG = {
            [stg("@ip_cmd", 0), stg("Rrpwr"), stg("Tg ip"), stg("LVPL LVACM", 1, {2, "@V_re"; 3, "@V_im"}), stg(">ip")]
            [stg("@iq_cmd", 0), stg("Iqrmax"), stg("Tg iq"), stg(">iq")]
            rowGen};
        gTitle = "Converter (WECC REGC_A)";
    end
    [y, h3, r3] = ctrlSection(mdl, gTitle, rowsG, x, y, cm, areaColor("asset"));
    hMid = [h1 h2 h3];  rMax = max([r1 r2 r3]);

    if isB
        % battery energy: dSOC/dt = -kSOC * P  (P in pu, + = discharge)
        nb(mdl, "gain", "kSOC", "Gain", "-ap.kSOC");
        nb(mdl, "int", "SOC int", "InitialCondition", "ap.SOC0");
        rowsB = {[stg("@P", 0), stg("kSOC"), stg("SOC int"), stg(">SOC")]};
        [~, h4, r4] = ctrlSection(mdl, "Battery energy", rowsB, x, y, cm, areaColor("asset"));
        hMid(end+1) = h4;  rMax = max(rMax, r4);
    end

    outs = ["P", "Q", "P_avail", "I_re", "I_im"];
    if isB, outs(end+1) = "SOC"; end
    hOut = ctrlOutputs(mdl, outs, "@" + outs, rMax + 110, cm);
    alignColumns(hIn, hMid, hOut);
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
    flag = ["memory", "delay:p.comm.meas_hppc"];  % no unit (SOC, saturation flags)
    pitch = 38;                                   % HPPC port spacing
    add_block("simulink/Ports & Subsystems/Model", sys + "/HPPC", "ModelName", "HPPC", ...
        "Position", [560 40 740 40 + 20*pitch]);
    Simulink.ModelReference.refresh(sys + "/HPPC");
    setArg(sys + "/HPPC", "par", "p.ctrl.hppc.par");
    set_param(sys + "/HPPC", "BackgroundColor", areaColor("controller"));

    % group: inport name, port number, bus elements, HPPC port per element, link steps per element
    G = { ...
      "TSO",       1, ["P_ref","Q_ref","f_meas"],             1:3,   {["gain:1/" + hb, "delay:p.comm.tso@p.grid.Pref/" + hb], ["gain:1/" + hb, "delay:p.comm.tso@p.grid.Qref/" + hb], strings(1,0)}; ...
      "POC meas",  2, ["P_poc","Q_poc","V_poc"],              4:6,   {meas, meas, ["memory", "delay:p.comm.meas_hppc@1"]}; ...
      "meas wpp",  3, ["P_avail","P","Q","Q_sat"],            7:10,  {meas, meas, meas, flag}; ...
      "meas spp",  4, ["P_avail","P","Q","Q_sat"],            11:14, {meas, meas, meas, flag}; ...
      "meas bess", 5, ["P","Q","P_chg_max","P_dis_max","SOC","Q_sat"], 15:20, {meas, meas, meas, meas, flag, flag}};
    I = modelInterfaces();  hppcIn = I.HPPC.in;   % port order of HPPC.slx (G above uses these numbers)
    for i = 1:size(G, 1)
        [name, port, elems, ports, steps] = G{i, :};
        % BESS: bus order P, Q, P_chg_max, P_dis_max, SOC, Q_sat -> HPPC ports 18, 19, 15, 16, 17, 20
        if name == "meas bess", ports = [18 19 15 16 17 20]; end
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
    nCin = 7 + isB;  nCout = 6 + 3*isB;
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
        mPorts = 4:9;  names = ["P", "Q", "P_chg_max", "P_dis_max", "SOC", "Q_sat"];
    else
        mPorts = 3:6;  names = ["P_avail", "P", "Q", "Q_sat"];
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
        if ismember(names(j), ["SOC", "Q_sat"])          % no unit conversion
            ln(sys, C + "/" + mPorts(j), "meas bus/" + j, names(j));
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
    % measurement memory), "delay:p.comm.<name>[@<initial output>]"
    % (communication delay, variable length p.comm.steps.<name>). Returns the
    % block name "<label> link" (not yet connected).
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
                % discrete delay of p.comm.steps.<name> whole simulation steps
                % (deriveParameters), as a variant chosen per run (delayMode)
                stepsField = "p.comm.steps." + extractAfter(parts(1), "p.comm.");
                buildDelayVariant(blk + "/" + bn, stepsField, init);
                lbl(end+1) = erase(parts(1), "p."); %#ok<AGROW>
                desc(end+1) = "delay " + parts(1) + " in whole steps (initial output " + init + ")"; %#ok<AGROW>
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

function buildDelayVariant(vs, stepsField, init)
    % Variant Subsystem with the two delay implementations as choices:
    %   Fixed    (HPP_delays == 1): delay length fixed at compile time, fastest
    %   Variable (HPP_delays == 2): length as a tunable input, up to
    %                               p.comm.maxDelay (buffer size), for sweeps
    % The variant control variable HPP_delays is set per run (delayMode via
    % runCase / runSweep / rapidTarget). Activation at update diagram: only
    % the active choice is compiled, so the fixed choice costs nothing extra.
    add_block("simulink/Ports & Subsystems/Variant Subsystem", vs);
    for b = string(find_system(vs, "SearchDepth", 1, "MatchFilter", @Simulink.match.allVariants, "Type", "Block"))'
        if b ~= vs, delete_block(b); end
    end
    L = find_system(vs, "SearchDepth", 1, "FindAll", "on", "Type", "line");
    if ~isempty(L), delete_line(L); end
    add_block("simulink/Sources/In1", vs + "/u", "Position", [40 50 70 64]);
    add_block("simulink/Sinks/Out1", vs + "/y", "Position", [420 50 450 64]);
    for c = ["Fixed", "Variable"]
        sub = vs + "/" + c;
        add_block("simulink/Ports & Subsystems/Subsystem", sub);
        delete_line(sub, "In1/1", "Out1/1");
        if c == "Fixed"
            add_block("simulink/Discrete/Delay", sub + "/Delay", "DelayLength", stepsField, ...
                "InitialCondition", init, "SampleTime", "p.sim.Ts");
        else
            add_block("simulink/Discrete/Delay", sub + "/Delay", "DelayLengthSource", "Input port", ...
                "DelayLengthUpperLimit", "round(p.comm.maxDelay/p.sim.Ts)", ...
                "InitialCondition", init, "SampleTime", "p.sim.Ts");
            add_block("simulink/Sources/Constant", sub + "/steps", "Value", stepsField, ...
                "SampleTime", "p.sim.Ts", "OutDataTypeStr", "double");
            add_line(sub, "steps/1", "Delay/2", "autorouting", "smart");
        end
        set_param(sub + "/In1", "Name", "u");          % choice ports match the container's by name
        set_param(sub + "/Out1", "Name", "y");
        add_line(sub, "u/1", "Delay/1", "autorouting", "smart");
        add_line(sub, "Delay/1", "y/1", "autorouting", "smart");
        Simulink.BlockDiagram.arrangeSystem(sub);
    end
    set_param(vs + "/Fixed", "VariantControl", "HPP_delays == 1", "Position", [180 20 300 60]);
    set_param(vs + "/Variable", "VariantControl", "HPP_delays == 2", "Position", [180 90 300 130]);
    set_param(vs, "VariantActivationTime", "update diagram", "Description", ...
        "Communication delay of " + stepsField + " steps, chosen per run: Fixed (HPP_delays = 1, " + ...
        "length fixed at compile time, fastest) or Variable (HPP_delays = 2, tunable up to " + ...
        "p.comm.maxDelay). See commParameters, comm.delays.");
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
%  Controller diagram layout (asset and plant controller templates)
%
%  Every controller is drawn in three columns of areas:
%    inputs (Inport -> Goto) | function areas, stacked | outputs (From -> Outport)
%  Inside a function area each signal path is one straight row, built by
%  rowChain: blocks left to right, each block's input aligned with the
%  previous block's output; other inputs come from local From blocks or
%  parameter constants placed directly left of their port; a row ends in
%  a Goto. Signals used in several rows are passed with Goto/From
%  (blue: references and orders, green: measurements, orange: internal).
%  ===================================================================
function st = stg(blk, ip, sides, outs)
    % One stage of a row: block blk with main input port ip (0: none).
    % sides = {port, src; ...}: src "@Tag" is a From, otherwise the name of
    % an existing block (a constant) placed left of the port.
    % outs = {port, tag; ...}: further outputs to Gotos.
    % blk "@Tag" creates a From (row start), ">Tag" a Goto (row end).
    if nargin < 2, ip = 1; end
    if nargin < 3, sides = {}; end
    if nargin < 4, outs = {}; end
    st = struct("blk", string(blk), "ip", ip, "sides", {sides}, "outs", {outs});
end

function [bottom, right, blks] = rowChain(sys, x, yTop, stages, cm)
    % Place one row of stages (see stg) with its top at yTop, starting at x;
    % returns the lowest edge, the right edge and all blocks of the row
    n = numel(stages);
    names = strings(1, n);  sideNames = cell(1, n);  outNames = cell(1, n);
    blks = strings(0);
    y = 0;  xc = x;
    for i = 1:n
        st = stages(i);  b = st.blk;
        if startsWith(b, "@")
            t = extractAfter(b, 1);  b = addFrom(sys, t, tagColor(cm, t));
        elseif startsWith(b, ">")
            t = extractAfter(b, 1);  b = addGoto(sys, t, tagColor(cm, t));
        end
        names(i) = b;  full = sys + "/" + b;
        sb = strings(1, size(st.sides, 1));  wmax = 0;
        for j = 1:size(st.sides, 1)
            s = string(st.sides{j, 2});
            if startsWith(s, "@")
                t = extractAfter(s, 1);  s = addFrom(sys, t, tagColor(cm, t));
            end
            sb(j) = s;
            p = get_param(sys + "/" + s, "Position");  wmax = max(wmax, p(3) - p(1));
        end
        gap = 35;
        if wmax > 0, gap = wmax + 50; end
        if i == 1, gap = 0; end
        p = get_param(full, "Position");  w = p(3) - p(1);  h = p(4) - p(2);
        xl = xc + gap;
        set_param(full, "Position", [xl, 0, xl + w, h]);
        if st.ip > 0
            dy = y - portY(full, "in", st.ip);
        else
            dy = y - portY(full, "out", 1);
        end
        set_param(full, "Position", [xl, dy, xl + w, dy + h]);
        for j = 1:numel(sb)
            sp = get_param(sys + "/" + sb(j), "Position");  sw = sp(3) - sp(1);  shh = sp(4) - sp(2);
            yc = portY(full, "in", st.sides{j, 1});
            set_param(sys + "/" + sb(j), "Position", round([xl - 20 - sw, yc - shh/2, xl - 20, yc + shh/2]));
        end
        ob = strings(1, size(st.outs, 1));
        for j = 1:size(st.outs, 1)
            t = string(st.outs{j, 2});
            ob(j) = addGoto(sys, t, tagColor(cm, t));
            gp = get_param(sys + "/" + ob(j), "Position");
            yc = portY(full, "out", st.outs{j, 1});
            set_param(sys + "/" + ob(j), "Position", round([xl + w + 30, yc - (gp(4)-gp(2))/2, ...
                xl + w + 30 + gp(3) - gp(1), yc + (gp(4)-gp(2))/2]));
        end
        sideNames{i} = sb;  outNames{i} = ob;
        ph = get_param(full, "PortHandles");
        if ~isempty(ph.Outport), y = portY(full, "out", 1); end
        xc = xl + w;
        blks = [blks, b, sb, ob]; %#ok<AGROW>
    end
    % shift the row to its place, then draw (straight) lines
    top = inf;  bottom = -inf;  right = -inf;
    for b = blks
        p = get_param(sys + "/" + b, "Position");
        top = min(top, p(2));  bottom = max(bottom, p(4));  right = max(right, p(3));
    end
    d = round(yTop - top);
    for b = blks
        set_param(sys + "/" + b, "Position", get_param(sys + "/" + b, "Position") + [0 d 0 d]);
    end
    bottom = bottom + d;
    for i = 1:n
        st = stages(i);
        if i > 1 && st.ip > 0
            add_line(sys, names(i-1) + "/1", names(i) + "/" + st.ip);
        end
        for j = 1:numel(sideNames{i})
            add_line(sys, sideNames{i}(j) + "/1", names(i) + "/" + st.sides{j, 1});
        end
        for j = 1:numel(outNames{i})
            add_line(sys, names(i) + "/" + st.outs{j, 1}, outNames{i}(j) + "/1");
        end
    end
end

function [y, h, right] = ctrlSection(sys, title, rows, x, y, cm, color)
    % Function area: rows (cell of stage arrays) stacked from y; returns the
    % y for the next section, the area handle and the right edge
    y = y + 40;                       % area title
    all = strings(0);  right = -inf;
    for r = 1:numel(rows)
        [b, rr, bl] = rowChain(sys, x, y, rows{r}, cm);
        y = b + 34;                   % room for block names
        all = [all, bl]; %#ok<AGROW>
        right = max(right, rr);
    end
    h = areaAround(sys, title, all, color);
    y = y + 10;
end

function h = areaAround(sys, title, blks, color)
    box = [inf inf -inf -inf];
    for b = blks
        p = get_param(sys + "/" + b, "Position");
        box = [min(box(1), p(1)), min(box(2), p(2)), max(box(3), p(3)), max(box(4), p(4))];
    end
    h = add_block("built-in/Area", sys + "/" + title, "Position", box + [-15 -36 15 26]);
    set_param(h, "BackgroundColor", color);
end

function alignColumns(hIn, hMid, hOut)
    % inputs and outputs areas span the full height; the function areas of
    % each column (hMid: one handle array per column) share their left and
    % right edges, and the last area of every column ends at the same height
    if ~iscell(hMid), hMid = {hMid}; end
    q1 = get_param(hIn, "Position");  q2 = get_param(hOut, "Position");
    top = min(q1(2), q2(2));  bot = max(q1(4), q2(4));
    for c = 1:numel(hMid)
        P = cell2mat(arrayfun(@(h) get_param(h, "Position"), hMid{c}(:), "UniformOutput", false));
        top = min(top, min(P(:, 2)));  bot = max(bot, max(P(:, 4)));
    end
    for c = 1:numel(hMid)
        h = hMid{c};
        P = cell2mat(arrayfun(@(x) get_param(x, "Position"), h(:), "UniformOutput", false));
        l = min(P(:, 1));  r = max(P(:, 3));
        for k = 1:numel(h)
            b = P(k, 4);
            if k == numel(h), b = bot; end
            set_param(h(k), "Position", [l, P(k, 2), r, b]);
        end
    end
    for h = [hIn, hOut]
        q = get_param(h, "Position");
        set_param(h, "Position", [q(1), top, q(3), bot]);
    end
end

function [hIn, xMid] = ctrlInputs(sys, names, dims, cm, ts)
    % Inport -> Goto for every input, in a column on the left (sample
    % time ts, default "Ts")
    if nargin < 5, ts = "Ts"; end
    y = 40;  all = strings(0);  right = 0;
    for k = 1:numel(names)
        add_block("simulink/Sources/In1", sys + "/" + names(k), "Port", num2str(k), ...
            "OutDataTypeStr", "double", "PortDimensions", dims(k), "SampleTime", ts, "Position", [0 0 30 14]);
        [b, rr, bl] = rowChain(sys, 40, y, [stg(names(k), 0), stg(">" + names(k))], cm);
        y = b + 34;  all = [all, bl]; %#ok<AGROW>
        right = max(right, rr);
    end
    hIn = areaAround(sys, "Inputs", all, areaColor("interface"));
    xMid = right + 90;
end

function hOut = ctrlOutputs(sys, names, srcs, x, cm)
    % From -> Outport for every output, in a column on the right; srcs are
    % Goto tags or names of existing source blocks
    y = 40;  all = strings(0);
    for k = 1:numel(names)
        add_block("simulink/Sinks/Out1", sys + "/" + names(k), "Port", num2str(k), ...
            "OutDataTypeStr", "double", "Position", [0 0 30 14]);
        s = srcs(k);
        if ~startsWith(s, "@") && getSimulinkBlockHandle(sys + "/" + s) == -1, s = "@" + s; end
        [b, ~, bl] = rowChain(sys, x, y, [stg(s, 0), stg(names(k))], cm);
        y = b + 34;  all = [all, bl]; %#ok<AGROW>
    end
    hOut = areaAround(sys, "Outputs", all, areaColor("interface"));
end

function c = tagColor(cm, tag)
    if any(cm.ref == tag)
        c = "[0.592 0.620 0.957]";      % blue: references, orders, commands
    elseif any(cm.meas == tag)
        c = "[0.122 0.816 0.510]";      % green: measurements
    else
        c = "[0.988 0.463 0.204]";      % orange: internal signals, flags
    end
end

function nb(sys, kind, name, varargin)
    % new block of a standard size (placed later by rowChain)
    switch kind
        case "sum",     lib = "simulink/Math Operations/Sum";            sz = [30 40];
            varargin = [varargin, {"IconShape", "rectangular"}];
        case "gain",    lib = "simulink/Math Operations/Gain";           sz = [50 30];
        case "div",     lib = "simulink/Math Operations/Divide";         sz = [35 40];
        case "minmax",  lib = "simulink/Math Operations/MinMax";         sz = [35 40];
        case "sumel",   lib = "simulink/Math Operations/Sum of Elements"; sz = [35 30];
        case "lag",     lib = "simulink/Discrete/Transfer Fcn First Order"; sz = [70 34];
        case "pid",     lib = "simulink/Discrete/Discrete PID Controller"; sz = [70 80];
        case "switch",  lib = "simulink/Signal Routing/Switch";           sz = [40 80];
        case "sat",     lib = "simulink/Discontinuities/Saturation";      sz = [40 30];
        case "satdyn",  lib = "simulink/Discontinuities/Saturation Dynamic"; sz = [50 80];
        case "rate",    lib = "simulink/Discontinuities/Rate Limiter";    sz = [40 30];
        case "dz",      lib = "simulink/Discontinuities/Dead Zone";       sz = [40 30];
        case "rel",     lib = "simulink/Logic and Bit Operations/Relational Operator"; sz = [35 40];
        case "or",      lib = "simulink/Logic and Bit Operations/Logical Operator";    sz = [35 40];
        case "term",    lib = "simulink/Sinks/Terminator";                sz = [20 20];
        case "mem",     lib = "simulink/Discrete/Memory";                 sz = [30 30];
        case "tf",      lib = "simulink/Continuous/Transfer Fcn";         sz = [70 34];
        case "int",     lib = "simulink/Continuous/Integrator";           sz = [30 30];
        case "dint",    lib = "simulink/Discrete/Discrete-Time Integrator"; sz = [30 30];
    end
    add_block(lib, sys + "/" + name, "Position", [0 0 sz], varargin{:});
end

function kc(sys, name, value)
    % parameter constant (shows its value; name hidden)
    w = max(40, 7*strlength(string(value)) + 14);
    add_block("simulink/Sources/Constant", sys + "/" + name, "Value", value, ...
        "Position", [0 0 w 24], "ShowName", "off");
end

function lagBlk(sys, name, T, y0)
    % first-order lag y(k) = p*y(k-1) + (1-p)*u(k), p = 1 - Ts/T, initial output y0
    nb(sys, "lag", name, "PoleZ", "1 - Ts/" + T, "ICPrevOutput", y0);
end

function contLag(sys, name, T)
    % continuous first-order lag 1/(T s + 1), initial output 0, built as
    % err -> gain 1/T -> integrator with feedback: the same as a Transfer Fcn
    % block, but its parameter "1/(T)" stays tunable in generated code (a
    % Transfer Fcn denominator "[T 1]" does not: vector concatenation)
    blk = sys + "/" + name;
    add_block("built-in/Subsystem", blk, "Position", [0 0 70 34]);
    add_block("built-in/Inport", blk + "/u", "Position", [20 43 50 57]);
    add_block("simulink/Math Operations/Sum", blk + "/err", "Inputs", "+-", "IconShape", "rectangular", ...
        "Position", [90 35 115 65]);
    add_block("simulink/Math Operations/Gain", blk + "/gain", "Gain", "1/(" + T + ")", "Position", [145 35 205 65]);
    add_block("simulink/Continuous/Integrator", blk + "/state", "Position", [235 35 265 65]);
    add_block("built-in/Outport", blk + "/y", "Position", [300 43 330 57]);
    add_line(blk, "u/1", "err/1");  add_line(blk, "err/1", "gain/1");
    add_line(blk, "gain/1", "state/1");  add_line(blk, "state/1", "y/1");
    add_line(blk, "state/1", "err/2", "autorouting", "on");
    set_param(blk, "Mask", "on", "MaskDisplay", "disp('1 / (Tg s + 1)')", "MaskIconOpaque", "off");
end

function piBlk(sys, name, Kp, Ki, lim)
    % discrete PI with clamping anti-windup; lim "" = external limits (up, lo ports)
    if lim == ""
        nb(sys, "pid", name, "Controller", "PI", "P", Kp, "I", Ki, "LimitOutput", "on", ...
            "SatLimitsSource", "external", "AntiWindupMode", "clamping", "SampleTime", "-1");
    else
        nb(sys, "pid", name, "Controller", "PI", "P", Kp, "I", Ki, "LimitOutput", "on", ...
            "UpperSaturationLimit", lim, "LowerSaturationLimit", "-" + lim, ...
            "AntiWindupMode", "clamping", "SampleTime", "-1");
    end
end

function ctrlHeader(mdl, c, desc, withN)
    closeIfLoaded(mdl);
    new_system(mdl);
    set_param(mdl, "SolverType", "Fixed-step", "Solver", "FixedStepDiscrete", ...
        "FixedStep", "Ts", "StopTime", "inf", "Description", desc);
    ws = get_param(mdl, "ModelWorkspace");
    ws.DataSource = "MATLAB Code";
    code = [
        "% Defaults for standalone use and code generation, from HPP_parameters."
        "% In HPP.slx the Model block passes p.ctrl." + c + ".par as the argument ""par""."
        "pDefault = HPP_parameters();"
        "Ts  = pDefault.ctrl." + c + ".Ts;    % [s] fixed sample time"
        "par = pDefault.ctrl." + c + ".par;   % tunable parameters (model argument)"];
    if withN
        code(end+1) = "N   = numel(par.share);   % asset instances (fixed per build)";
    end
    code(end+1) = "clear pDefault";
    ws.MATLABCode = char(strjoin(code, newline));
    reload(ws);
    set_param(mdl, "ParameterArgumentNames", "par");
end

function ctrlSave(mdl, folder)
    save_system(mdl, fullfile(folder, mdl + ".slx"));
    close_system(mdl);
    registerInProject(fullfile(folder, mdl + ".slx"));
end

%% ===================================================================
%  WT_ctrl template: IEC 61400-27-1 Type 4A control (stage 3). Structure
%  after IEC 61400-27-1 (measurement, P control type 4A, Q control,
%  current limitation); block details and parameter values to be checked
%  against the standard. Ports:
%    in  1 P_set  2 Q_set  3 V_term  4 P_meas  5 Q_meas  6 P_wind_avail
%    out 1 ip_cmd 2 iq_cmd 3 P_avail          (pu on the asset rating)
%  ===================================================================
function buildWTCtrlType4A(folder)
    mdl = "WT_ctrl";
    ctrlHeader(mdl, "wt", ...
        "WT asset controller: IEC 61400-27-1 Type 4A control (measurement, P control type 4A, Q control " + ...
        "with closed-loop reactive power control (MqG) and dip current injection, current limitation). " + ...
        "Per unit on the asset rating. Parameters: model argument par (= p.ctrl.wt.par, from " + ...
        "p.wt.iec.ctrl). Structure after the standard; details and values to be checked against it.", false);
    cm.ref  = ["P_set", "Q_set", "P_wind_avail", "ip_req", "iq_req", "ip_cmd", "iq_cmd"];
    cm.meas = ["V_term", "P_meas", "Q_meas", "U", "U_floor", "Q_filt"];
    [hIn, x] = ctrlInputs(mdl, ["P_set", "Q_set", "V_term", "P_meas", "Q_meas", "P_wind_avail"], ...
        repmat("1", 1, 6), cm);

    % measurement
    lagBlk(mdl, "U filt", "par.Tufilt", "1");
    lagBlk(mdl, "P filt", "par.Tpfilt", "0");
    lagBlk(mdl, "Q filt", "par.Tqfilt", "0");
    kc(mdl, "Vmin", "par.Vmin");   nb(mdl, "minmax", "U floor", "Function", "max", "Inputs", "2");
    kc(mdl, "udip", "par.udip");   nb(mdl, "rel", "dip", "Operator", "<");
    nb(mdl, "term", "P filt unused", "ShowName", "off");   % no P loop in Type 4A
    rowsM = {
        [stg("@V_term", 0), stg("U filt"), stg(">U")]
        [stg("@U", 0), stg("U floor", 1, {2, "Vmin"}), stg(">U_floor")]
        [stg("@U", 0), stg("dip", 1, {2, "udip"}), stg(">dip")]
        [stg("@Q_meas", 0), stg("Q filt"), stg(">Q_filt")]
        [stg("@P_meas", 0), stg("P filt"), stg("P filt unused")]};
    [y, h1, r1] = ctrlSection(mdl, "Measurement and dip detection", rowsM, x, 20, cm, areaColor("interface"));

    % P control type 4A
    kc(mdl, "P min", "0");
    nb(mdl, "satdyn", "P_ref limit");
    nb(mdl, "rate", "dpmaxp4A", "RisingSlewLimit", "par.dpmaxp4A", "FallingSlewLimit", "-par.dpmaxp4A");
    lagBlk(mdl, "Tpordp4A", "par.Tpordp4A", "0");
    nb(mdl, "div", "p_ord to ip");
    rowsP = {[stg("@P_set", 0), stg("P_ref limit", 2, {1, "@P_wind_avail"; 3, "P min"}), stg("dpmaxp4A"), ...
              stg("Tpordp4A"), stg("p_ord to ip", 1, {2, "@U_floor"}), stg(">ip_req")]};
    [y, h2, r2] = ctrlSection(mdl, "P control type 4A", rowsP, x, y, cm, areaColor("controller"));

    % Q control: open loop q_ref / U plus PI (MqG = 1), dip current
    nb(mdl, "div", "q_ref to iq");
    nb(mdl, "sum", "iq ff + PI", "Inputs", "++");
    nb(mdl, "sum", "q error", "Inputs", "+-");
    kc(mdl, "MqG = 1", "par.MqG1");  kc(mdl, "no q loop", "0");   % MqG1 = (MqG == 1), from controllerParameters
    nb(mdl, "switch", "q loop on?", "Criteria", "u2 ~= 0");
    kc(mdl, "q frozen", "0");
    nb(mdl, "switch", "freeze in dip", "Criteria", "u2 ~= 0");
    piBlk(mdl, "kpq kiq", "par.kpq", "par.kiq", "par.imax");
    kc(mdl, "uref0", "par.uref0");
    nb(mdl, "sum", "uref0 - U", "Inputs", "+-");
    nb(mdl, "gain", "Kqv", "Gain", "par.Kqv");
    nb(mdl, "sum", "iq + dip", "Inputs", "++");
    nb(mdl, "switch", "in dip?", "Criteria", "u2 ~= 0");
    nb(mdl, "sat", "iqH1", "UpperLimit", "par.iqH1", "LowerLimit", "-par.iqH1");
    rowsQ = {
        [stg("@Q_set", 0), stg("q_ref to iq", 1, {2, "@U_floor"}), stg("iq ff + PI", 1, {2, "@iq_PI"}), stg(">iq_ol")]
        [stg("@Q_set", 0), stg("q error", 1, {2, "@Q_filt"}), stg("q loop on?", 1, {2, "MqG = 1"; 3, "no q loop"}), ...
         stg("freeze in dip", 3, {1, "q frozen"; 2, "@dip"}), stg("kpq kiq"), stg(">iq_PI")]
        [stg("@U", 0), stg("uref0 - U", 2, {1, "uref0"}), stg("Kqv"), stg("iq + dip", 2, {1, "@iq_ol"}), ...
         stg("in dip?", 1, {2, "@dip"; 3, "@iq_ol"}), stg("iqH1"), stg(">iq_req")]};
    [y, h3, r3] = ctrlSection(mdl, "Q control (MqG 1 closed loop, 2 open loop) and dip current", rowsQ, x, y, cm, areaColor("controller"));

    % current limitation
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
        ["ip_cmd", "iq_cmd"], body, "par", [0 0 150 110]);
    rowsC = {[stg("@ip_req", 0), stg("current limitation", 1, {2, "@iq_req"; 3, "@dip"}, {2, "iq_cmd"}), stg(">ip_cmd")]};
    [~, h4, r4] = ctrlSection(mdl, "Current limitation", rowsC, x, y, cm, areaColor("asset"));

    % outputs (Type 4A: constant aerodynamic power, P_avail = P_wind_avail)
    hOut = ctrlOutputs(mdl, ["ip_cmd", "iq_cmd", "P_avail"], ["@ip_cmd", "@iq_cmd", "@P_wind_avail"], ...
        max([r1 r2 r3 r4]) + 110, cm);
    alignColumns(hIn, [h1 h2 h3 h4], hOut);
    ctrlSave(mdl, folder);
end

%% ===================================================================
%  PV_ctrl / BESS_ctrl template: WECC second-generation electrical
%  control (stage 4). PV: REEC_B; BESS: REEC_C (REEC_B structure plus
%  state-of-charge limits). Qflag 1: closed-loop Q (PI), 0: open loop; no
%  voltage-dependent current limit (VDL), no state freeze during a dip
%  except the Q PI. Ports:
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
    ctrlHeader(mdl, c, ...
        type + " asset controller: WECC " + std + " electrical control (voltage filter and dip " + ...
        "detection, P path with ramp and Tpord lag, Q path with Qflag (closed-loop Q PI) and dip current " + ...
        "injection Kqv, current limit logic with PQflag" + extra + "). Per unit on the asset rating. " + ...
        "Parameters: model argument par (= p.ctrl." + c + ".par, from p." + c + ".wecc.ctrl). " + ...
        "Simplified; details and values to be checked against the WECC specification.", false);
    cm.ref  = ["P_set", "Q_set", "P_pv_avail", "ip_req", "iq_req", "ip_cmd", "iq_cmd"];
    cm.meas = ["V_term", "P_meas", "Q_meas", "V", "V_floor", "SOC"];
    [hIn, x] = ctrlInputs(mdl, ["P_set", "Q_set", "V_term", "P_meas", "Q_meas", src], repmat("1", 1, 6), cm);

    % voltage filter and dip detection
    lagBlk(mdl, "Vt filt", "par.Trv", "1");
    kc(mdl, "Vmin", "par.Vmin");  nb(mdl, "minmax", "V floor", "Function", "max", "Inputs", "2");
    kc(mdl, "Vdip", "par.Vdip");  nb(mdl, "rel", "V < Vdip", "Operator", "<");
    kc(mdl, "Vup", "par.Vup");    nb(mdl, "rel", "V > Vup", "Operator", ">");
    nb(mdl, "or", "Voltage_dip", "Operator", "OR");
    nb(mdl, "term", "P_meas unused", "ShowName", "off");   % no P loop in REEC_B/C
    rowsM = {
        [stg("@V_term", 0), stg("Vt filt"), stg(">V")]
        [stg("@V", 0), stg("V floor", 1, {2, "Vmin"}), stg(">V_floor")]
        [stg("@V", 0), stg("V > Vup", 1, {2, "Vup"}), stg(">V_high")]
        [stg("@V", 0), stg("V < Vdip", 1, {2, "Vdip"}), stg("Voltage_dip", 1, {2, "@V_high"}), stg(">dip")]
        [stg("@P_meas", 0), stg("P_meas unused")]};
    [y, h1, r1] = ctrlSection(mdl, "Voltage filter and dip detection", rowsM, x, 20, cm, areaColor("interface"));

    % P path
    if isB
        kc(mdl, "Pmax", "par.Pmax");  kc(mdl, "Pmin", "-par.Pmax");  up = "Pmax";
    else
        kc(mdl, "Pmin", "0");  up = "@P_pv_avail";
    end
    nb(mdl, "satdyn", "Pmax Pmin");
    nb(mdl, "rate", "dPmax dPmin", "RisingSlewLimit", "par.dPmax", "FallingSlewLimit", "par.dPmin");
    lagBlk(mdl, "Tpord", "par.Tpord", "0");
    nb(mdl, "div", "Pord to Ip");
    rowsP = {[stg("@P_set", 0), stg("Pmax Pmin", 2, {1, up; 3, "Pmin"}), stg("dPmax dPmin"), stg("Tpord"), ...
              stg("Pord to Ip", 1, {2, "@V_floor"}), stg(">ip_req")]};
    [y, h2, r2] = ctrlSection(mdl, "P path", rowsP, x, y, cm, areaColor("controller"));

    % Q path: Qext / V plus PI (Qflag = 1), lag Tiq, reactive current injection in a dip
    nb(mdl, "div", "Qext to Iq");
    nb(mdl, "sum", "ff + PI", "Inputs", "++");
    lagBlk(mdl, "Tiq", "par.Tiq", "0");
    nb(mdl, "sum", "Q error", "Inputs", "+-");
    kc(mdl, "Qflag", "par.Qflag");  kc(mdl, "no Q loop", "0");
    nb(mdl, "switch", "Q loop on?", "Criteria", "u2 ~= 0");
    kc(mdl, "Q frozen", "0");
    nb(mdl, "switch", "freeze in dip", "Criteria", "u2 ~= 0");
    piBlk(mdl, "Kqp Kqi", "par.Kqp", "par.Kqi", "par.Imax");
    kc(mdl, "Vref0", "par.Vref0");
    nb(mdl, "sum", "Vref0 - V", "Inputs", "+-");
    nb(mdl, "dz", "dbd1 dbd2", "LowerValue", "par.dbd1", "UpperValue", "par.dbd2");
    nb(mdl, "gain", "Kqv", "Gain", "par.Kqv");
    nb(mdl, "sat", "Iqh1 Iql1", "UpperLimit", "par.Iqh1", "LowerLimit", "par.Iql1");
    kc(mdl, "no Iqinj", "0");
    nb(mdl, "switch", "Iqinj in dip", "Criteria", "u2 ~= 0");
    nb(mdl, "sum", "Iq + Iqinj", "Inputs", "++");
    rowsQ = {
        [stg("@Q_set", 0), stg("Qext to Iq", 1, {2, "@V_floor"}), stg("ff + PI", 1, {2, "@iq_PI"}), stg("Tiq"), ...
         stg("Iq + Iqinj", 1, {2, "@iq_inj"}), stg(">iq_req")]
        [stg("@Q_set", 0), stg("Q error", 1, {2, "@Q_meas"}), stg("Q loop on?", 1, {2, "Qflag"; 3, "no Q loop"}), ...
         stg("freeze in dip", 3, {1, "Q frozen"; 2, "@dip"}), stg("Kqp Kqi"), stg(">iq_PI")]
        [stg("@V", 0), stg("Vref0 - V", 2, {1, "Vref0"}), stg("dbd1 dbd2"), stg("Kqv"), stg("Iqh1 Iql1"), ...
         stg("Iqinj in dip", 1, {2, "@dip"; 3, "no Iqinj"}), stg(">iq_inj")]};
    [y, h3, r3] = ctrlSection(mdl, "Q path (Qflag 1 closed loop, 0 open loop) and reactive current injection", ...
        rowsQ, x, y, cm, areaColor("controller"));

    % current limit logic
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
    cin = ["ip_req", "iq_req"];  sides = {2, "@iq_req"};
    if isB, cin(end+1) = "SOC"; sides(end+1, :) = {3, "@SOC"}; end
    addMatlabFunction(mdl + "/current limit logic", "current_limit_logic", cin, ...
        ["ip_cmd", "iq_cmd"], body, "par", [0 0 150 40*numel(cin) + 20]);
    rowsC = {[stg("@ip_req", 0), stg("current limit logic", 1, sides, {2, "iq_cmd"}), stg(">ip_cmd")]};
    [~, h4, r4] = ctrlSection(mdl, "Current limit logic", rowsC, x, y, cm, areaColor("asset"));

    % outputs: P_avail = available PV power, or the BESS converter limit
    if isB
        kc(mdl, "Pmax out", "par.Pmax");  pa = "Pmax out";
    else
        pa = "@P_pv_avail";
    end
    hOut = ctrlOutputs(mdl, ["ip_cmd", "iq_cmd", "P_avail"], ["@ip_cmd", "@iq_cmd", pa], max([r1 r2 r3 r4]) + 110, cm);
    alignColumns(hIn, [h1 h2 h3 h4], hOut);
    ctrlSave(mdl, folder);
end

%% ===================================================================
%  HPPC template (stage 5): closed-loop P at the POC with the frequency
%  response of the whole plant (central control), dispatch to the plants
%  (wind and PV at available power, the BESS balances; wind and PV are
%  curtailed only when the BESS cannot absorb the surplus), and Q or
%  voltage control at the POC over the plants' own Q loops (Q order split
%  by rating). Ports (pu on p.ctrl.hppc.Pbase):
%    in   1 P_ref 2 Q_ref 3 f_meas 4 P_poc 5 Q_poc 6 V_poc
%         7 P_avail_wpp 8 P_wpp 9 Q_wpp 10 Q_sat_wpp
%         11 P_avail_spp 12 P_spp 13 Q_spp 14 Q_sat_spp
%         15 P_chg_max_bess 16 P_dis_max_bess 17 SOC_bess 18 P_bess 19 Q_bess 20 Q_sat_bess
%    out  1 P_set_wpp 2 Q_set_wpp 3 P_set_spp 4 Q_set_spp 5 P_set_bess 6 Q_set_bess
%  ===================================================================
function buildHppcCtrl(folder)
    mdl = "HPPC";
    ctrlHeader(mdl, "hppc", ...
        "HPP controller (stage 5, Energinet DK1): P set-point with ramp rate constraint, frequency " + ...
        "response (LFSM-O, LFSM-U, optional FSM with reserve; Frqflag) added after the ramp limit, export " + ...
        "limit; closed-loop P at the POC (PI on P_target - P_poc, target fed forward, Pflag); SOC-aware " + ...
        "BESS limits; dispatch: wind and PV at available power, the BESS balances, wind and PV curtailed " + ...
        "when the BESS cannot absorb more. Q control (Q_ref), voltage control (Vref, slope Vslope) or " + ...
        "power factor control (PF) at the POC with PI (Qflag, RefFlag 0 / 1 / 2), Q order split over the " + ...
        "plants by rating. Both PIs frozen in a voltage dip. " + ...
        "Per unit on p.ctrl.hppc.Pbase. Parameters: model argument " + ...
        "par (= p.ctrl.hppc.par, from p.hppc).", false);
    I = modelInterfaces();                    % the one definition of the HPPC ports
    ins = I.HPPC.in;  outs = I.HPPC.out;
    cm.ref  = ["P_ref", "Q_ref", "P_base", "P_target", "P_ord", "Q_refx", "Q_ff", "Q_ord", outs];
    cm.meas = ["f_meas", "P_poc", "Q_poc", "V_poc", "P_avail_wpp", "P_wpp", "Q_wpp", "P_avail_spp", ...
               "P_spp", "Q_spp", "P_chg_max_bess", "P_dis_max_bess", "SOC_bess", "P_bess", "Q_bess", ...
               "P_filt", "Q_filt", "V_filt", "P_hi", "P_lo", "Q_sat_wpp", "Q_sat_spp", "Q_sat_bess"];
    [hIn, x] = ctrlInputs(mdl, ins, repmat("1", 1, numel(ins)), cm);

    % measurements, SOC-aware BESS limits and plant P limits
    lagBlk(mdl, "P filt", "par.Tp", "0");
    body = [
        "% SOC-aware BESS limits: discharge (charge) derated linearly to zero"
        "% over SOCband above SOCmin (below SOCmax), so the BESS approaches its"
        "% SOC limits smoothly instead of stopping at them."
        "kd = min(max((SOC_bess - par.SOCmin) / par.SOCband, 0), 1);"
        "kc = min(max((par.SOCmax - SOC_bess) / par.SOCband, 0), 1);"
        "BESS_dis = P_dis_max_bess * kd;"
        "BESS_chg = P_chg_max_bess * kc;"];
    addMatlabFunction(mdl + "/SOC limits", "soc_limits", ["P_dis_max_bess", "P_chg_max_bess", "SOC_bess"], ...
        ["BESS_dis", "BESS_chg"], body, "par", [0 0 140 100]);
    nb(mdl, "sum", "available", "Inputs", "+++");
    set_param(mdl + "/available", "Position", [0 0 30 80]);
    nb(mdl, "gain", "charge limit", "Gain", "-1");
    body = [
        "% Freeze the P loop while the POC voltage is below Vfrz (voltage dip)"
        "% and for Thld after it, while the assets recover their active power."
        "persistent n"
        "if isempty(n), n = 0; end"
        "if V_poc < par.Vfrz"
        "    n = round(par.Thld / Ts);"
        "elseif n > 0"
        "    n = n - 1;"
        "end"
        "frz = double(V_poc < par.Vfrz || n > 0);"];
    addMatlabFunction(mdl + "/dip freeze", "dip_freeze", ["V_poc", "Ts"], "frz", body, "par", [0 0 120 70]);
    kc(mdl, "Ts", "Ts");
    lagBlk(mdl, "Q filt", "par.Tq", "0");
    lagBlk(mdl, "V filt", "par.Tq", "1");
    rowsM = {
        [stg("@P_poc", 0), stg("P filt"), stg(">P_filt")]
        [stg("@Q_poc", 0), stg("Q filt"), stg(">Q_filt")]
        [stg("@V_poc", 0), stg("V filt"), stg(">V_filt")]
        [stg("@V_poc", 0), stg("dip freeze", 1, {2, "Ts"}), stg(">frz")]
        [stg("@P_dis_max_bess", 0), stg("SOC limits", 1, {2, "@P_chg_max_bess"; 3, "@SOC_bess"}, {2, "BESS_chg"}), ...
         stg(">BESS_dis")]
        [stg("@P_avail_wpp", 0), stg("available", 1, {2, "@P_avail_spp"; 3, "@BESS_dis"}), stg(">P_hi")]
        [stg("@BESS_chg", 0), stg("charge limit"), stg(">P_lo")]};
    [y, h1, r1] = ctrlSection(mdl, "Measurements, SOC-aware BESS limits, P limits", rowsM, x, 20, cm, ...
        areaColor("interface"));

    % P set-point with ramp limit, frequency response (Energinet: frequency
    % control before power and ramp constraints, so the response is added
    % after the ramp limiter), export limit
    body = [
        "% Base P order: the TSO set-point, limited to what the plant can deliver"
        "% while keeping the FSM reserve, with the ramp rate constraint (rampUp,"
        "% rampDn in pu of Pn per minute). Starts at the set-point."
        "persistent y"
        "if isempty(y), y = P_ref; end"
        "u = min(P_ref, max(P_hi - reserve, 0));"
        "if par.rampFlag"
        "    y = y + min(max(u - y, -par.rampDn / 60 * Ts), par.rampUp / 60 * Ts);"
        "else"
        "    y = u;"
        "end"
        "P_base = y;"];
    addMatlabFunction(mdl + "/ramp limiter", "ramp_limiter", ["P_ref", "P_hi", "reserve", "Ts"], "P_base", body, ...
        "par", [0 0 130 120]);
    kc(mdl, "FSM reserve", "par.Frqflag * par.FSMflag * par.fsmRange");
    kc(mdl, "Ts 2", "Ts");
    body = [
        "% Active power frequency response, pu of Pn (= POC base) per pu of f:"
        "% LFSM-O above fO (droop sO) and LFSM-U below fU (droop sU), always on;"
        "% FSM (FSMflag, FCR delivery): droop sFSM with deadband fsmDb, limited"
        "% to +-fsmRange. Frqflag 0 switches all of it off (decentral cases)."
        "df = f_meas - par.Freq_ref;"
        "dP = 0;"
        "if par.FSMflag"
        "    e = sign(df) * max(abs(df) - par.fsmDb, 0);"
        "    dP = min(max(-e / par.sFSM, -par.fsmRange), par.fsmRange);"
        "end"
        "dP = dP - max(f_meas - par.fO, 0) / par.sO + max(par.fU - f_meas, 0) / par.sU;"
        "dP_f = par.Frqflag * dP;"];
    addMatlabFunction(mdl + "/frequency response", "frequency_response", "f_meas", "dP_f", body, "par", [0 0 130 50]);
    nb(mdl, "sum", "base + freq", "Inputs", "++");
    kc(mdl, "Ppoc_max", "par.Ppoc_max");
    nb(mdl, "minmax", "export limit", "Function", "min", "Inputs", "2");
    rowsF = {
        [stg("@P_ref", 0), stg("ramp limiter", 1, {2, "@P_hi"; 3, "FSM reserve"; 4, "Ts 2"}), stg(">P_base")]
        [stg("@f_meas", 0), stg("frequency response"), stg(">dP_f")]
        [stg("@P_base", 0), stg("base + freq", 1, {2, "@dP_f"}), stg("export limit", 1, {2, "Ppoc_max"}), ...
         stg(">P_target")]};
    [y, h2, r2] = ctrlSection(mdl, "Ramp limit, frequency response (LFSM-O, LFSM-U, FSM), export limit", rowsF, ...
        x, y, cm, areaColor("controller"));

    % P control at the POC (Pflag): PI on P_target - P_filt, target fed forward
    nb(mdl, "sum", "Pmax - target", "Inputs", "+-");
    nb(mdl, "sum", "Pmin - target", "Inputs", "+-");
    lagBlk(mdl, "Tref", "par.Tref", "0");   % reference filter: target as the plant can follow it
    nb(mdl, "sum", "P error", "Inputs", "+-");
    kc(mdl, "Pflag", "par.Pflag");  kc(mdl, "no P loop", "0");
    nb(mdl, "switch", "P loop on?", "Criteria", "u2 ~= 0");
    kc(mdl, "frozen", "0");
    nb(mdl, "switch", "freeze in dip", "Criteria", "u2 ~= 0");
    nb(mdl, "sat", "femax femin", "UpperLimit", "par.femax", "LowerLimit", "par.femin");
    piBlk(mdl, "Kpg Kig", "par.Kpg", "par.Kig", "");
    nb(mdl, "sum", "target + PI", "Inputs", "++");
    nb(mdl, "satdyn", "Pmax Pmin");
    rowsP = {
        [stg("@P_hi", 0), stg("Pmax - target", 1, {2, "@P_target"}), stg(">P_up_PI")]
        [stg("@P_lo", 0), stg("Pmin - target", 1, {2, "@P_target"}), stg(">P_lo_PI")]
        [stg("@P_target", 0), stg("Tref"), stg("P error", 1, {2, "@P_filt"}), stg("P loop on?", 1, {2, "Pflag"; 3, "no P loop"}), ...
         stg("freeze in dip", 3, {1, "frozen"; 2, "@frz"}), stg("femax femin"), ...
         stg("Kpg Kig", 1, {2, "@P_up_PI"; 3, "@P_lo_PI"}), stg(">P_PI")]
        [stg("@P_target", 0), stg("target + PI", 1, {2, "@P_PI"}), stg("Pmax Pmin", 2, {1, "@P_hi"; 3, "@P_lo"}), ...
         stg(">P_ord")]};
    [y, h3, r3] = ctrlSection(mdl, "P control at the POC (Pflag)", rowsP, x, y, cm, areaColor("controller"));

    % Q / V control at the POC (Qflag, RefFlag): PI on the Q error (Q_ref
    % filtered by TrefQ - Q_filt, Q_ref fed forward) or on the voltage error
    % (Vref - V_filt - Kc Q_filt, no feed-forward), frozen in a dip
    nb(mdl, "gain", "Kc", "Gain", "par.Vslope / par.Qmax");   % slope: Vslope for a Q change of Qmax
    nb(mdl, "sum", "V + Kc Q", "Inputs", "++");
    kc(mdl, "Vref", "par.Vref");
    nb(mdl, "sum", "V error", "Inputs", "+-");
    body = [
        "% Q limit at the POC in voltage control: where the slope asks for more"
        "% than Qmax (less than Qmin), the voltage error is capped by the Q margin,"
        "% scaled by the loop gain (V error per Q), so the loop settles at the"
        "% limit. Needed with local voltage control in the plants, where the HPPC"
        "% order is only the offset of their droop lines."
        "k = par.Vslope / par.Qmax + par.dVdQ;"
        "V_errL = min(max(V_err, (par.Qmin - Q_filt) * k), (par.Qmax - Q_filt) * k);"];
    addMatlabFunction(mdl + "/POC Q limit", "poc_q_limit", ["V_err", "Q_filt"], "V_errL", body, "par", [0 0 110 60]);
    % block parameters with model arguments must stay simple arithmetic (+ - * /)
    % to be tunable in generated code (accelerator, DLL): derived values such as
    % tan(phi) and the mode flags come from controllerParameters
    nb(mdl, "gain", "tan phi", "Gain", "par.tanphi");
    kc(mdl, "PF mode", "par.isPF");
    nb(mdl, "switch", "PF control?", "Criteria", "u2 ~= 0");
    kc(mdl, "RefFlag ff", "par.isV");  kc(mdl, "no Q ff", "0");
    nb(mdl, "switch", "Q feed-forward", "Criteria", "u2 ~= 0");
    % order limits: Qmax / Qmin, or wider when the order is only the offset of
    % the plants' local V-Q droop lines (the plants limit their real Q)
    kc(mdl, "Qmax", "par.QmaxOrd");  nb(mdl, "sum", "Qmax - ff", "Inputs", "+-");
    kc(mdl, "Qmin", "par.QminOrd");  nb(mdl, "sum", "Qmin - ff", "Inputs", "+-");
    lagBlk(mdl, "TrefQ", "par.TrefQ", "0");
    nb(mdl, "sum", "Q error", "Inputs", "+-");
    kc(mdl, "RefFlag", "par.isV");
    nb(mdl, "switch", "V control?", "Criteria", "u2 ~= 0");
    kc(mdl, "Qflag", "par.Qflag");  kc(mdl, "no Q loop", "0");
    nb(mdl, "switch", "Q loop on?", "Criteria", "u2 ~= 0");
    kc(mdl, "Q frozen", "0");
    nb(mdl, "switch", "freeze Q in dip", "Criteria", "u2 ~= 0");
    nb(mdl, "sat", "eqmax eqmin", "UpperLimit", "par.eqmax", "LowerLimit", "par.eqmin");
    % voltage control: gains normalised by the loop gain LV (from
    % controllerParameters: with local voltage control in the plants it is
    % the slow trim of their Q offsets, otherwise direct POC voltage control)
    body = [
        "% Anti-windup over the plant limits (conditional integration): the HPPC"
        "% order is split over all plants, so when every plant reports its Q order"
        "% at the upper (lower) limit, more (less) order cannot change Q: hold the"
        "% Q loop (error 0) in that direction. Plants' own limits are inside the"
        "% plant controllers; this is how the HPPC learns about them."
        "up = Q_sat_wpp > 0.5 && Q_sat_spp > 0.5 && Q_sat_bess > 0.5;"
        "dn = Q_sat_wpp < -0.5 && Q_sat_spp < -0.5 && Q_sat_bess < -0.5;"
        "e_aw = e;"
        "if par.QawFlag && ((e > 0 && up) || (e < 0 && dn))"
        "    e_aw = 0;"
        "end"];
    addMatlabFunction(mdl + "/plant anti-windup", "plant_anti_windup", ["e", "Q_sat_wpp", "Q_sat_spp", "Q_sat_bess"], ...
        "e_aw", body, "par", [0 0 130 140]);
    piBlk(mdl, "Q PI", "par.KpQ", "par.KiQ", "");   % Q / PF or (normalised) voltage gains, controllerParameters
    nb(mdl, "sum", "ff + Q PI", "Inputs", "++");
    nb(mdl, "sat", "Qmax Qmin", "UpperLimit", "par.QmaxOrd", "LowerLimit", "par.QminOrd");
    rowsQ = {
        [stg("@Q_filt", 0), stg("Kc"), stg(">KcQ")]
        [stg("@V_filt", 0), stg("V + Kc Q", 1, {2, "@KcQ"}), stg("V error", 2, {1, "Vref"}), ...
         stg("POC Q limit", 1, {2, "@Q_filt"}), stg(">V_err")]
        [stg("@P_filt", 0), stg("tan phi"), stg("PF control?", 1, {2, "PF mode"; 3, "@Q_ref"}), stg(">Q_refx")]
        [stg("@Q_refx", 0), stg("Q feed-forward", 3, {1, "no Q ff"; 2, "RefFlag ff"}), stg(">Q_ff")]
        [stg("Qmax", 0), stg("Qmax - ff", 1, {2, "@Q_ff"}), stg(">Q_up_PI")]
        [stg("Qmin", 0), stg("Qmin - ff", 1, {2, "@Q_ff"}), stg(">Q_lo_PI")]
        [stg("@Q_refx", 0), stg("TrefQ"), stg("Q error", 1, {2, "@Q_filt"}), stg("V control?", 3, {1, "@V_err"; 2, "RefFlag"}), ...
         stg("Q loop on?", 1, {2, "Qflag"; 3, "no Q loop"}), stg("freeze Q in dip", 3, {1, "Q frozen"; 2, "@frz"}), ...
         stg("eqmax eqmin"), stg("plant anti-windup", 1, {2, "@Q_sat_wpp"; 3, "@Q_sat_spp"; 4, "@Q_sat_bess"}), ...
         stg("Q PI", 1, {2, "@Q_up_PI"; 3, "@Q_lo_PI"}), stg(">Q_PI")]
        [stg("@Q_ff", 0), stg("ff + Q PI", 1, {2, "@Q_PI"}), stg("Qmax Qmin"), stg(">Q_ord")]};
    x2 = max([r1 r2 r3]) + 80;   % second column of function areas
    [y2, h6, r6] = ctrlSection(mdl, "Q, V or PF control at the POC (Qflag, RefFlag 0 1 2)", rowsQ, x2, 20, cm, areaColor("controller"));

    % dispatch to the plants
    body = [
        "% P: wind and PV at available power, the BESS balances the rest within"
        "% its SOC-aware charge / discharge limits; wind and PV are curtailed (in"
        "% proportion to their availability) only when the BESS cannot absorb the"
        "% surplus (also when it is nearly full)."
        "% Q: the HPPC's Q order split over the plants by rating."
        "P_ren = P_avail_wpp + P_avail_spp;"
        "P_set_bess = min(max(P_ord - P_ren, -BESS_chg), BESS_dis);"
        "P_r = min(max(P_ord - P_set_bess, 0), P_ren);"
        "if P_ren > 1e-6"
        "    w = P_avail_wpp / P_ren;"
        "else"
        "    w = par.qshare(1) / (par.qshare(1) + par.qshare(2));"
        "end"
        "P_set_wpp = P_r * w;"
        "P_set_spp = P_r * (1 - w);"
        "Q_set_wpp  = Q_ord * par.qshare(1);"
        "Q_set_spp  = Q_ord * par.qshare(2);"
        "Q_set_bess = Q_ord * par.qshare(3);"];
    din = ["P_ord", "Q_ord", "P_avail_wpp", "P_avail_spp", "BESS_chg", "BESS_dis"];
    addMatlabFunction(mdl + "/dispatch", "dispatch", din, outs, body, "par", [0 0 160 40*numel(din) + 20]);
    rowsD = {[stg("@P_ord", 0), stg("dispatch", 1, {2, "@Q_ord"; 3, "@P_avail_wpp"; 4, "@P_avail_spp"; ...
        5, "@BESS_chg"; 6, "@BESS_dis"}, {2, "Q_set_wpp"; 3, "P_set_spp"; 4, "Q_set_spp"; ...
        5, "P_set_bess"; 6, "Q_set_bess"}), stg(">P_set_wpp")]};
    [y2, h4, r4] = ctrlSection(mdl, "Dispatch to the plants", rowsD, x2, y2, cm, areaColor("asset"));

    % inputs not used yet (plant measurements: later functions)
    unused = ["P_wpp", "Q_wpp", "P_spp", "Q_spp", "P_bess", "Q_bess"];
    rowsU = cell(numel(unused), 1);
    for k = 1:numel(unused)
        nb(mdl, "term", unused(k) + " unused", "ShowName", "off");
        rowsU{k} = [stg("@" + unused(k), 0), stg(unused(k) + " unused")];
    end
    [~, h5, r5] = ctrlSection(mdl, "Not used yet", rowsU, x2, y2, cm, areaColor("interface"));

    hOut = ctrlOutputs(mdl, outs, "@" + outs, max([r1 r2 r3 r4 r5 r6]) + 110, cm);
    alignColumns(hIn, {[h1 h2 h3], [h6 h4 h5]}, hOut);
    ctrlSave(mdl, folder);
end

%% ===================================================================
%  WPPC / SPPC / BESSC template: WECC REPC_A plant controller, closing its
%  own loops on the plant feeder into the 33 kV busbar (vendor-style plant
%  controller under the HPPC): P with frequency droop and PI (Frqflag),
%  Q or V with PI (RefFlag), references fed forward, then dispatch over
%  the N assets. Ports:
%    in  1 P_set 2 Q_set 3 P_meas 4 Q_meas 5 V_meas 6 P_avail_k [7 SOC_k] last f_meas
%    out 1 P_set_k 2 Q_set_k 3 P_avail 4 P_plant 5 Q_plant
%        [6 P_chg_max 7 P_dis_max 8 SOC]       (pu on the plant rating)
%        last: Q_sat (+1 / -1: Q order at its upper / lower limit; for the
%        HPPC's anti-windup)
%  ===================================================================
function buildPlantCtrlRepc(name, folder)
    mdl = string(name);
    isB = mdl == "BESSC";
    c = lower(mdl);
    ctrlHeader(mdl, c, ...
        mdl + " plant controller: WECC REPC_A structure (P control with frequency droop and PI, Q or " + ...
        "voltage control with PI, references fed forward), closed loop on the plant feeder into the 33 kV " + ...
        "busbar, then dispatch over the N assets by availability. Per unit on the plant rating. " + ...
        "Parameters: model argument par (= p.ctrl." + c + ".par, from p.pc." + c + ").", true);
    cm.ref  = ["P_set", "Q_set", "P_ref", "P_ord", "Q_ord", "P_set_k", "Q_set_k", "Q_ff"];
    cm.meas = ["P_meas", "Q_meas", "V_meas", "f_meas", "P_avail_k", "SOC_k", "P_filt", "Q_filt", "V_filt", ...
               "P_avail", "P_chg_max", "P_dis_max", "SOC"];
    ins = ["P_set", "Q_set", "P_meas", "Q_meas", "V_meas", "P_avail_k"];
    dimsIn = ["1", "1", "1", "1", "1", "N"];
    if isB, ins(end+1) = "SOC_k"; dimsIn(end+1) = "N"; end
    ins(end+1) = "f_meas"; dimsIn(end+1) = "1";
    [hIn, x] = ctrlInputs(mdl, ins, dimsIn, cm);

    % measurements and availability / P limits
    lagBlk(mdl, "P filt", "par.Tp", "0");
    lagBlk(mdl, "Q filt", "par.Tfltr", "0");
    lagBlk(mdl, "V filt", "par.Tfltr", "1");
    nb(mdl, "sumel", "P_avail sum");
    rowsM = {
        [stg("@P_meas", 0), stg("P filt"), stg(">P_filt")]
        [stg("@Q_meas", 0), stg("Q filt"), stg(">Q_filt")]
        [stg("@V_meas", 0), stg("V filt"), stg(">V_filt")]
        [stg("@P_avail_k", 0), stg("P_avail sum"), stg(">P_avail")]};
    if isB
        body = [
            "% plant charge / discharge limits: only assets within their SOC range"
            "dis = P_avail_k .* (SOC_k > par.SOCmin);"
            "chg = P_avail_k .* (SOC_k < par.SOCmax);"
            "P_dis_max = sum(dis);"
            "P_chg_max = sum(chg);"
            "SOC = sum(par.share .* SOC_k);"];
        addMatlabFunction(mdl + "/BESS limits", "bess_limits", ["P_avail_k", "SOC_k"], ...
            ["P_chg_max", "P_dis_max", "SOC"], body, "par", [0 0 140 120]);
        nb(mdl, "gain", "charge limit", "Gain", "-1");
        rowsM = [rowsM
            {[stg("@P_avail_k", 0), stg("BESS limits", 1, {2, "@SOC_k"}, {2, "P_dis_max"; 3, "SOC"}), stg(">P_chg_max")]}
            {[stg("@P_chg_max", 0), stg("charge limit"), stg(">P_lo")]}];
        up = "P_dis_max";
    else
        kc(mdl, "Pmin", "0");
        rowsM = [rowsM; {[stg("Pmin", 0), stg(">P_lo")]}];
        up = "P_avail";
    end
    [y, h1, r1] = ctrlSection(mdl, "Measurements and P limits", rowsM, x, 20, cm, areaColor("interface"));

    % frequency response: P_ref = P_set + droop
    kc(mdl, "Freq_ref", "par.Freq_ref");
    nb(mdl, "sum", "f error", "Inputs", "+-");
    nb(mdl, "dz", "fdbd", "LowerValue", "par.fdbd1", "UpperValue", "par.fdbd2");
    nb(mdl, "sat", "under-f", "UpperLimit", "inf", "LowerLimit", "0");
    nb(mdl, "sat", "over-f", "UpperLimit", "0", "LowerLimit", "-inf");
    nb(mdl, "gain", "Dup", "Gain", "par.Dup");
    nb(mdl, "gain", "Ddn", "Gain", "par.Ddn");
    nb(mdl, "sum", "P_set + droop", "Inputs", "+++");
    set_param(mdl + "/P_set + droop", "Position", [0 0 30 80]);
    rowsF = {
        [stg("@f_meas", 0), stg("f error", 2, {1, "Freq_ref"}), stg("fdbd"), stg(">df")]
        [stg("@df", 0), stg("under-f"), stg("Dup"), stg(">dP_up")]
        [stg("@df", 0), stg("over-f"), stg("Ddn"), stg(">dP_dn")]
        [stg("@P_set", 0), stg("P_set + droop", 1, {2, "@dP_up"; 3, "@dP_dn"}), stg(">P_ref")]};
    [y, h2, r2] = ctrlSection(mdl, "Frequency response (Ddn, Dup)", rowsF, x, y, cm, areaColor("controller"));

    % P control (Frqflag): PI on P_ref - P_filt, P_ref fed forward
    nb(mdl, "sum", "Pmax - ref", "Inputs", "+-");
    nb(mdl, "sum", "Pmin - ref", "Inputs", "+-");
    nb(mdl, "sum", "P error", "Inputs", "+-");
    nb(mdl, "sat", "femax femin", "UpperLimit", "par.femax", "LowerLimit", "par.femin");
    piBlk(mdl, "Kpg Kig", "par.Kpg", "par.Kig", "");
    nb(mdl, "sum", "ref + PI", "Inputs", "++");
    kc(mdl, "Frqflag", "par.Frqflag");
    nb(mdl, "switch", "closed loop?", "Criteria", "u2 ~= 0");
    nb(mdl, "satdyn", "Pmax Pmin");
    lagBlk(mdl, "Tlag", "par.Tlag", "0");
    rowsP = {
        [stg("@" + up, 0), stg("Pmax - ref", 1, {2, "@P_ref"}), stg(">P_up_PI")]
        [stg("@P_lo", 0), stg("Pmin - ref", 1, {2, "@P_ref"}), stg(">P_lo_PI")]
        [stg("@P_ref", 0), stg("P error", 1, {2, "@P_filt"}), stg("femax femin"), ...
         stg("Kpg Kig", 1, {2, "@P_up_PI"; 3, "@P_lo_PI"}), stg(">P_PI")]
        [stg("@P_ref", 0), stg("ref + PI", 1, {2, "@P_PI"}), stg("closed loop?", 1, {2, "Frqflag"; 3, "@P_set"}), ...
         stg("Pmax Pmin", 2, {1, "@" + up; 3, "@P_lo"}), stg("Tlag"), stg(">P_ord")]};
    [y, h3, r3] = ctrlSection(mdl, "P control (Frqflag)", rowsP, x, y, cm, areaColor("controller"));

    % Q or V control (RefFlag): PI on the Q error, or local voltage control on
    % a V-Q droop line whose Q offset is the HPPC set-point:
    % Vref - V - Kc (Q - Q_set) = 0, i.e. Q = Q_set + (Vref - V) / Kc.
    % Q_set is fed forward in both modes. Voltage-control gains are normalised
    % by the loop gain Kc + dVdQ (dVdQ: busbar dV/dQ of all plants together,
    % on the own rating), so the speed hardly depends on the grid.
    nb(mdl, "sum", "Q - Q_set", "Inputs", "+-");
    nb(mdl, "gain", "Kc", "Gain", "par.Kc");
    nb(mdl, "sum", "V + Kc Q", "Inputs", "++");
    kc(mdl, "Vref", "par.Vref");
    nb(mdl, "sum", "V error", "Inputs", "+-");
    kc(mdl, "Vfrz", "par.Vfrz");
    nb(mdl, "rel", "V < Vfrz", "Operator", "<");
    kc(mdl, "RefFlag ff", "0");  kc(mdl, "no ff", "0");   % Q_set fed forward in both modes
    nb(mdl, "switch", "Q feed-forward", "Criteria", "u2 ~= 0");
    kc(mdl, "Qmax", "par.Qmax");  nb(mdl, "sum", "Qmax - ff", "Inputs", "+-");
    kc(mdl, "Qmin", "par.Qmin");  nb(mdl, "sum", "Qmin - ff", "Inputs", "+-");
    nb(mdl, "sum", "Q error", "Inputs", "+-");
    kc(mdl, "RefFlag", "par.RefFlag");
    nb(mdl, "switch", "V control?", "Criteria", "u2 ~= 0");
    nb(mdl, "dz", "dbd", "LowerValue", "par.dbd1", "UpperValue", "par.dbd2");
    nb(mdl, "sat", "emax emin", "UpperLimit", "par.emax", "LowerLimit", "par.emin");
    kc(mdl, "frozen", "0");
    nb(mdl, "switch", "freeze", "Criteria", "u2 ~= 0");
    piBlk(mdl, "Kp Ki", "par.Kp + par.RefFlag*(par.KpV / (par.Kc + par.dVdQ) - par.Kp)", ...
        "par.Ki + par.RefFlag*(par.KiV / (par.Kc + par.dVdQ) - par.Ki)", "");
    nb(mdl, "sum", "ff + PI", "Inputs", "++");
    lagBlk(mdl, "Tfv", "par.Tfv", "0");
    % saturation status for the HPPC's anti-windup: +1 / -1 when the Q order
    % is at its upper / lower limit (the plant cannot give more / less)
    body = [
        "tol = 1e-4;"
        "Q_sat = double(Q_u >= par.Qmax - tol) - double(Q_u <= par.Qmin + tol);"];
    addMatlabFunction(mdl + "/Q saturation", "q_saturation", "Q_u", "Q_sat", body, "par", [0 0 110 40]);
    rowsQ = {
        [stg("@Q_filt", 0), stg("Q - Q_set", 1, {2, "@Q_set"}), stg("Kc"), stg(">KcQ")]
        [stg("@V_filt", 0), stg("V + Kc Q", 1, {2, "@KcQ"}), stg("V error", 2, {1, "Vref"}), stg(">V_err")]
        [stg("@V_filt", 0), stg("V < Vfrz", 1, {2, "Vfrz"}), stg(">V_low")]
        [stg("@Q_set", 0), stg("Q feed-forward", 3, {1, "no ff"; 2, "RefFlag ff"}), stg(">Q_ff")]
        [stg("Qmax", 0), stg("Qmax - ff", 1, {2, "@Q_ff"}), stg(">Q_up_PI")]
        [stg("Qmin", 0), stg("Qmin - ff", 1, {2, "@Q_ff"}), stg(">Q_lo_PI")]
        [stg("@Q_set", 0), stg("Q error", 1, {2, "@Q_filt"}), stg("V control?", 3, {1, "@V_err"; 2, "RefFlag"}), ...
         stg("dbd"), stg("emax emin"), stg("freeze", 3, {1, "frozen"; 2, "@V_low"}), ...
         stg("Kp Ki", 1, {2, "@Q_up_PI"; 3, "@Q_lo_PI"}), stg(">Q_PI")]
        [stg("@Q_ff", 0), stg("ff + PI", 1, {2, "@Q_PI"}, {1, "Q_u"}), stg("Tfv"), stg(">Q_ord")]
        [stg("@Q_u", 0), stg("Q saturation"), stg(">Q_sat")]};
    x2 = max([r1 r2 r3]) + 80;   % second column of function areas
    [y, h4, r4] = ctrlSection(mdl, "Q or V control (RefFlag)", rowsQ, x2, 20, cm, areaColor("controller"));

    % dispatch over the assets
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
        [0 0 150 40*numel(din) + 20]);
    sides = {2, "@Q_ord"; 3, "@P_avail_k"};
    if isB, sides(end+1, :) = {4, "@SOC_k"}; end
    rowsD = {[stg("@P_ord", 0), stg("dispatch", 1, sides, {2, "Q_set_k"}), stg(">P_set_k")]};
    [~, h5, r5] = ctrlSection(mdl, "Dispatch to the assets", rowsD, x2, y, cm, areaColor("asset"));

    % outputs
    outs = ["P_set_k", "Q_set_k", "P_avail", "P_plant", "Q_plant"];
    srcs = ["@P_set_k", "@Q_set_k", "@P_avail", "@P_meas", "@Q_meas"];
    if isB
        outs = [outs, "P_chg_max", "P_dis_max", "SOC"];
        srcs = [srcs, "@P_chg_max", "@P_dis_max", "@SOC"];
    end
    outs = [outs, "Q_sat"];  srcs = [srcs, "@Q_sat"];   % last output: Q order at its limit (+1 / -1)
    hOut = ctrlOutputs(mdl, outs, srcs, max([r1 r2 r3 r4 r5]) + 110, cm);
    alignColumns(hIn, {[h1 h2 h3], [h4 h5]}, hOut);
    ctrlSave(mdl, folder);
end

%% ===================================================================
%  Network as a Simscape Electrical model (branch grid-components)
%
%  Same ports as buildNetwork: in 1..3 current injections per plant
%  (buses I_re, I_im, system pu); out 1 network bus (V_re, V_im of every
%  asset terminal one step old, P_plant / Q_plant [W / var], V_bus, f),
%  out 2 POC bus (P_poc, Q_poc, V_poc).
%  Inside: standard three-phase blocks solved in frequency-and-time mode
%  (phasors around 50 Hz), one Solver Configuration:
%    converter interface -- unit impedance (series R-L, unit transformer
%    referred to the collector voltage) -- pi feeder segments -- plant
%    node (wind: 10/33 kV transformer) -- plant meter -- 33 kV HPP bus --
%    33/132 kV transformer -- POC meter -- grid (E behind Zg, E with dips)
%  Layout from p.<type>.nFeeders / nPerFeeder and p.net (networkParameters);
%  block parameters are expressions in p, so case values apply.
%  ===================================================================
function buildNetworkVariants(sys, p)
    % Variant Subsystem "Network" with the two network models as choices:
    %   Nodal    (HPP_network == 1): nodal solver Z = inv(Y), fast
    %   Simscape (HPP_network == 2): Simscape Electrical, frequency-and-time mode
    % Both have the same ports. The variant control variable HPP_network is
    % set per run by runCase (networkCode(p.net.solverType)); the default for
    % runs from the editor is in the model workspace. Activation at update
    % diagram: the inactive choice is not compiled.
    add_block("simulink/Ports & Subsystems/Variant Subsystem", sys);
    for b = string(find_system(sys, "SearchDepth", 1, "MatchFilter", @Simulink.match.allVariants, "Type", "Block"))'
        if b ~= sys, delete_block(b); end
    end
    L = find_system(sys, "SearchDepth", 1, "FindAll", "on", "Type", "line");
    if ~isempty(L), delete_line(L); end
    buildNetwork(sys + "/Nodal");
    buildNetworkSimscape(sys + "/Simscape", p);
    set_param(sys + "/Nodal", "VariantControl", "HPP_network == 1", "Position", [200 50 400 250]);
    set_param(sys + "/Simscape", "VariantControl", "HPP_network == 2", "Position", [200 300 400 500]);
    tags = ["wpp", "spp", "bess"];
    for i = 1:3
        add_block("simulink/Sources/In1", sys + "/I " + tags(i), "Port", num2str(i), "Position", [40 40*i 70 40*i+14]);
    end
    add_block("simulink/Sources/In1", sys + "/f grid", "Port", "4", "Position", [40 160 70 174]);
    add_block("simulink/Sources/In1", sys + "/theta", "Port", "5", "Position", [40 200 70 214]);
    add_block("simulink/Sinks/Out1", sys + "/network", "Port", "1", "Position", [520 60 550 74]);
    add_block("simulink/Sinks/Out1", sys + "/POC meas", "Port", "2", "Position", [520 120 550 134]);
    set_param(sys, "VariantActivationTime", "update diagram", "Description", ...
        "Network model, chosen per run: Nodal (HPP_network = 1, fast nodal solver) or Simscape " + ...
        "(HPP_network = 2, Simscape Electrical in frequency-and-time mode). See runCase, option Network.");
end

function buildGridFrequency(sys)
    % Grid frequency f [pu] and grid source angle theta [rad] (stage 6):
    % imposed (p.grid.fev step + p.grid.fprof profile) or a swing equation
    % of an aggregated system behind the grid impedance (p.grid.sys.swing =
    % 1), driven by a disturbance and the change of P_poc. theta rotates the
    % grid source in the network (frame at f0). Masked: parameter g =
    % p.grid (tunable). In: POC measurements (P_poc, one step old). Out:
    % 1 f, 2 theta.
    newSubsystem(sys);
    add_block("simulink/Sources/In1", sys + "/POC meas", "Port", "1", "Position", [20 40 50 54]);
    add_block("simulink/Signal Routing/Bus Selector", sys + "/POC sel", "Position", [100 30 105 70]);
    set_param(sys + "/POC sel", "OutputSignals", "P_poc");
    add_block("simulink/Discrete/Memory", sys + "/P_poc prev", "InitialCondition", "0", "Position", [150 35 180 65]);
    add_block("simulink/Sources/Digital Clock", sys + "/t", "SampleTime", "p.sim.Ts", "Position", [140 100 180 130]);
    add_block("simulink/Sources/Constant", sys + "/Ts", "Value", "p.sim.Ts", "SampleTime", "p.sim.Ts", "Position", [140 160 180 190]);
    body = [
        "% state: frequency deviation w, governor xg, mechanical power Pm, angle"
        "persistent w xg Pm Phold th"
        "if isempty(w), w = 0; xg = 0; Pm = 0; Phold = P_poc; th = 0; end"
        "s = g.sys;"
        "if s.swing > 0.5"
        "    if t < s.tOn"
        "        Phold = P_poc;                       % balanced at tOn"
        "    else"
        "        dPh = (P_poc - Phold) / s.Ssys;       % HPP change, pu on Ssys"
        "        dPd = s.dist.dP * (t >= s.dist.t0);"
        "        dw  = (Pm + dPh - dPd - s.D * w) / (2 * s.H);"
        "        dxg = (-w / s.R - xg) / s.Tg;"
        "        dPm = (xg - Pm) / s.Tt;"
        "        w = w + Ts * dw;  xg = xg + Ts * dxg;  Pm = Pm + Ts * dPm;"
        "    end"
        "    f = 1 + w;"
        "else"
        "    f = 1 + g.fev.df * (t >= g.fev.t0) + interp1(g.fprof.t, g.fprof.df, t, 'linear', 0);"
        "end"
        "th = th + 2 * pi * g.f0 * (f - 1) * Ts;"
        "th = mod(th + pi, 2 * pi) - pi;"
        "theta = th;"];
    addMatlabFunction(sys + "/frequency", "grid_frequency", ["P_poc", "t", "Ts"], ["f", "theta"], body, "g", [240 40 400 190]);
    add_block("simulink/Sinks/Out1", sys + "/f", "Port", "1", "Position", [470 60 500 74]);
    add_block("simulink/Sinks/Out1", sys + "/theta", "Port", "2", "Position", [470 150 500 164]);
    ln(sys, "POC meas/1", "POC sel/1", "");
    ln(sys, "POC sel/1", "P_poc prev/1", "P_poc");
    ln(sys, "P_poc prev/1", "frequency/1", "");
    ln(sys, "t/1", "frequency/2", "");
    ln(sys, "Ts/1", "frequency/3", "");
    ln(sys, "frequency/1", "f/1", "f");
    ln(sys, "frequency/2", "theta/1", "theta");
    mk = Simulink.Mask.create(sys);
    mk.addParameter("Name", "g", "Prompt", "Grid data", "Value", "p.grid", "Evaluate", "on", "Tunable", "on");
    mk.Description = "Grid frequency [pu] and angle [rad]: imposed (p.grid.fev, p.grid.fprof) or " + ...
        "swing equation (p.grid.sys.swing = 1). See params/gridParameters.m.";
    mk.Display = "disp('grid frequency')";
    set_param(sys, "BackgroundColor", areaColor("interface"));
end

function buildNetworkSimscape(sys, p)
    newSubsystem(sys);
    load_system("ee_lib");
    lib = eeLibrary();
    types = ["wt", "pv", "bess"];  tags = ["wpp", "spp", "bess"];
    titles = ["Wind plant: converters, unit impedances, 10 kV feeders, 10-33 kV transformer", ...
              "PV plant: converters, unit impedances, 33 kV feeders", "BESS: converter, unit impedance, cable"];
    N = arrayfun(@(t) p.(t).N, types);  nA = sum(N);
    H = @(b) get_param(sys + "/" + b, "PortHandles");
    PL = @(a, b) add_line(sys, a, b);                     % physical line, straight when aligned
    f0 = "p.grid.f0";
    cm.ref  = ["Ire_" + (1:nA), "Iim_" + (1:nA), "Ire_" + tags, "Iim_" + tags, "t", "E_mag", "E_ph"];
    cm.meas = ["Vre_" + (1:nA), "Vim_" + (1:nA), "V_re_prev", "V_im_prev", "P_" + tags, "Q_" + tags, ...
               "P_plant", "Q_plant", "V_bus", "P_poc", "Q_poc", "V_poc"];
    hMid = [];

    % ================= inputs: current injections and time =================
    for t = 1:3
        add_block("simulink/Sources/In1", sys + "/I " + tags(t), "Port", num2str(t), "Position", [0 0 30 14]);
        add_block("simulink/Signal Routing/Bus Selector", sys + "/I sel " + tags(t), "OutputSignals", "I_re,I_im", ...
            "ShowName", "off", "Position", [0 0 6 50]);
    end
    add_block("simulink/Sources/Digital Clock", sys + "/clock", "SampleTime", "p.sim.Ts", "Position", [0 0 40 24]);
    rows = {};  k0 = 0;
    for t = 1:3
        n = N(t);  ks = k0 + (1:n);
        if n > 1
            add_block("simulink/Signal Routing/Demux", sys + "/Ire " + tags(t), "Outputs", num2str(n), "ShowName", "off", "Position", [0 0 6 40*n]);
            add_block("simulink/Signal Routing/Demux", sys + "/Iim " + tags(t), "Outputs", num2str(n), "ShowName", "off", "Position", [0 0 6 40*n]);
            outsRe = [num2cell((2:n)'), num2cell("Ire_" + ks(2:end)')];
            outsIm = [num2cell((2:n)'), num2cell("Iim_" + ks(2:end)')];
            rows{end+1} = [stg("I " + tags(t), 0), stg("I sel " + tags(t), 1, {}, {2, "Iim_" + tags(t)}), stg(">Ire_" + tags(t))]; %#ok<AGROW>
            rows{end+1} = [stg("@Ire_" + tags(t), 0), stg("Ire " + tags(t), 1, {}, outsRe), stg(">Ire_" + ks(1))]; %#ok<AGROW>
            rows{end+1} = [stg("@Iim_" + tags(t), 0), stg("Iim " + tags(t), 1, {}, outsIm), stg(">Iim_" + ks(1))]; %#ok<AGROW>
        else
            rows{end+1} = [stg("I " + tags(t), 0), stg("I sel " + tags(t), 1, {}, {2, "Iim_" + ks(1)}), stg(">Ire_" + ks(1))]; %#ok<AGROW>
        end
        k0 = k0 + n;
    end
    rows{end+1} = [stg("clock", 0), stg(">t")];
    [~, hIn, rIn] = ctrlSection(sys, "Current injections (system pu) and time", rows, 40, 20, cm, areaColor("interface"));

    % ================= plants: one row per converter =================
    xC = rIn + 120;  y = 20;  k = 0;  rMax = 0;
    for t = 1:3
        q = p.(types(t));  tt = types(t);  pn = "p.net." + tt;
        hasTr = p.net.(tt).plant.S > 0;
        cab = pn + ".cable";
        Ib = "p.grid.Pbase/(sqrt(3)*" + pn + ".Vcol)";
        Zu = "(" + pn + ".unit.uk*" + pn + ".Vcol^2/(" + pn + ".unit.S*p." + tt + ".nAgg))";
        blks = strings(0);  y = y + 40;  yPlant = [];
        for f = 1:q.nFeeders
            for i = 1:q.nPerFeeder
                k = k + 1;  s = sprintf("%d", k);
                convBlock(sys, "converter " + s, Ib, pn + ".Vcol", [0 0 140 120], lib);
                [b, ~, bl] = rowChain(sys, xC, y, [stg("@Ire_" + s, 0), stg("converter " + s, 1, {2, "@Iim_" + s; 3, "@t"}, {2, "Vim_" + s}), ...
                    stg(">Vre_" + s)], cm);
                blks = [blks, bl]; %#ok<AGROW>
                % physical row: converter ~ -- unit Z -- feeder segment -- node label
                cp = get_param(H("converter " + s).RConn(1), "Position");
                add_block(lib("Passive/RLC Assemblies/RLC (Three-Phase)"), sys + "/unit Z " + s, "component_structure", ...
                    "ee.enum.rlc.structure.SeriesRL", "R_conf", "runtime", "L_conf", "runtime", "R", Zu + "/sqrt(1+" + pn + ".unit.XR^2)", ...
                    "L", Zu + "*" + pn + ".unit.XR/sqrt(1+" + pn + ".unit.XR^2)/(2*pi*" + f0 + ")", "Position", [0 0 60 40]);
                alignPhys(sys + "/unit Z " + s, cp(1) + 170, cp(2));
                if i == 1, len = cab + ".len1"; else, len = cab + ".len"; end
                piBlock(sys, "feeder " + s, cab, len, f0, [0 0 80 50], lib);
                zp = get_param(H("unit Z " + s).RConn(1), "Position");
                alignPhys(sys + "/feeder " + s, zp(1) + 80, zp(2));
                PL(H("converter " + s).RConn(1), H("unit Z " + s).LConn(1));
                PL(H("unit Z " + s).RConn(1), H("feeder " + s).LConn(1));
                % far end of this segment = tap of unit k; near end = previous tap or the collector
                tl = "";
                if i < q.nPerFeeder
                    tl = tapLabel(sys, "tap_" + s, H("feeder " + s).LConn(1));
                end
                if i == 1
                    near = tags(t) + "_collector";
                else
                    near = "tap_" + (k-1);
                end
                lb = nodeLabel(sys, near, H("feeder " + s).RConn(1), "right");
                blks = [blks, "unit Z " + s, "feeder " + s, lb]; %#ok<AGROW>
                if tl ~= "", blks(end+1) = tl; end %#ok<AGROW>
                yPlant(end+1) = zp(2); %#ok<AGROW>
                low = b;
                for x_ = ["unit Z " + s, "feeder " + s, tl]
                    if x_ ~= "", pp_ = get_param(sys + "/" + x_, "Position"); low = max(low, pp_(4)); end
                end
                y = low + 50;
            end
        end
        % plant column: collector label (-- plant transformer) -- meter -- HPP bus label
        fp = get_param(H("feeder " + (k - q.N + 1)).RConn(1), "Position");
        xP = fp(1) + 200;  yP = round(mean(yPlant));
        meterBlock(sys, tags(t) + " meter", [0 0 90 80], lib);
        if hasTr
            trBlock(sys, tags(t) + " transformer", pn + ".plant", pn + ".Vcol", "p.net.Vhpp", [0 0 90 70], lib);
            alignPhys(sys + "/" + tags(t) + " transformer", xP + 60, yP);
            lc = nodeLabel(sys, tags(t) + "_collector", H(tags(t) + " transformer").LConn(1), "left");
            tp = get_param(H(tags(t) + " transformer").RConn(1), "Position");
            alignPhys(sys + "/" + tags(t) + " meter", tp(1) + 50, tp(2));
            PL(H(tags(t) + " transformer").RConn(1), H(tags(t) + " meter").LConn(1));
            blks = [blks, tags(t) + " transformer", lc]; %#ok<AGROW>
        else
            alignPhys(sys + "/" + tags(t) + " meter", xP + 60, yP);
            lc = nodeLabel(sys, tags(t) + "_collector", H(tags(t) + " meter").LConn(1), "left");
            blks(end+1) = lc; %#ok<AGROW>
        end
        lb = nodeLabel(sys, "HPP_bus", H(tags(t) + " meter").RConn(1), "right");
        mp = get_param(sys + "/" + tags(t) + " meter", "Position");
        pq = ["P_", "Q_"] + tags(t);
        for j = 1:2
            g = addGoto(sys, pq(j), tagColor(cm, pq(j)));
            gp = get_param(sys + "/" + g, "Position");
            yo = portY(sys + "/" + tags(t) + " meter", "out", j);
            set_param(sys + "/" + g, "Position", [mp(3) + 120, yo - 11, mp(3) + 120 + gp(3) - gp(1), yo + 11]);
            add_line(sys, tags(t) + " meter/" + j, g + "/1", "autorouting", "on");
            blks(end+1) = g; %#ok<AGROW>
        end
        blks = [blks, tags(t) + " meter", lb]; %#ok<AGROW>
        h = areaAround(sys, titles(t), blks, areaColor("asset"));
        hMid(end+1) = h; %#ok<AGROW>
        r = get_param(h, "Position");  rMax = max(rMax, r(3));
        y = r(4) + 20;
    end

    % ================= grid column =================
    % (1) grid voltage magnitude with dips (p.grid.dip), line rms
    xH = rMax + 80;
    add_block("simulink/Sources/Constant", sys + "/E0", "Value", "p.grid.E", "Position", [0 0 50 24], "ShowName", "off");
    add_block("simulink/Sources/Step", sys + "/dip start", "Time", "p.grid.dip.t0", "Before", "0", ...
        "After", "p.grid.dip.depth", "SampleTime", "0", "Position", [0 0 30 30]);
    add_block("simulink/Sources/Step", sys + "/dip end", "Time", "p.grid.dip.t0 + p.grid.dip.dur", "Before", "0", ...
        "After", "p.grid.dip.depth", "SampleTime", "0", "Position", [0 0 30 30]);
    nb(sys, "sum", "E grid", "Inputs", "+-+");  set_param(sys + "/E grid", "Position", [0 0 30 110]);
    nb(sys, "gain", "to V", "Gain", "p.grid.Vpoc");
    add_block("simulink/Sources/In1", sys + "/theta", "Port", "5", "Position", [0 0 30 14]);   % grid angle (Grid frequency)
    rows = {[stg("E0", 0), stg("E grid", 1, {2, "dip start"; 3, "dip end"}), stg("to V"), stg(">E_mag")]
            [stg("theta", 0), stg(">E_ph")]};
    [yG, hDip] = ctrlSection(sys, "Grid voltage E with dips (line rms) and phase", rows, xH + 40, 20, cm, areaColor("interface"));

    % (2) HPP bus -- HPP transformer -- POC meter -- Zg -- E
    yH = yG + 120;  blks = strings(0);
    trBlock(sys, "HPP transformer", "p.net.main", "p.net.Vhpp", "p.grid.Vpoc", [0 0 100 80], lib);
    alignPhys(sys + "/HPP transformer", xH + 120, yH);
    blks(end+1) = nodeLabel(sys, "HPP_bus", H("HPP transformer").LConn(1), "left");
    meterBlock(sys, "POC meter", [0 0 90 80], lib);
    tp = get_param(H("HPP transformer").RConn(1), "Position");
    alignPhys(sys + "/POC meter", tp(1) + 60, tp(2));
    PL(H("HPP transformer").RConn(1), H("POC meter").LConn(1));
    Zg = "(p.grid.Vpoc^2/(p.grid.SCR*p.grid.Pmax_poc))";
    add_block(lib("Passive/RLC Assemblies/RLC (Three-Phase)"), sys + "/grid Zg", "component_structure", ...
        "ee.enum.rlc.structure.SeriesRL", "R_conf", "runtime", "L_conf", "runtime", "R", Zg + "/sqrt(1+p.grid.XR^2)", ...
        "L", Zg + "*p.grid.XR/sqrt(1+p.grid.XR^2)/(2*pi*" + f0 + ")", "Position", [0 0 60 40]);
    mp = get_param(H("POC meter").RConn(1), "Position");
    alignPhys(sys + "/grid Zg", mp(1) + 180, mp(2));
    PL(H("POC meter").RConn(1), H("grid Zg").LConn(1));
    add_block(lib("Sources/Controlled Voltage Source (Three-Phase)"), sys + "/grid E", "input_option", ...
        "ee.enum.inputType.singleFreqAC", "FRated", f0, "Orientation", "left", "Position", [0 0 80 80]);
    zp = get_param(H("grid Zg").RConn(1), "Position");
    ep = get_param(sys + "/grid E", "Position");  er = get_param(H("grid E").RConn(1), "Position");
    set_param(sys + "/grid E", "Position", ep + [zp(1) + 60 - ep(1), zp(2) - er(2), zp(1) + 60 - ep(1), zp(2) - er(2)]);
    PL(H("grid Zg").RConn(1), H("grid E").RConn(1));
    add_block(sprintf("ee_lib/Connectors &\nReferences/Electrical Reference"), sys + "/gE", "Position", [0 0 20 20]);
    np = get_param(H("grid E").LConn(3), "Position");
    set_param(sys + "/gE", "Position", [np(1) + 30, np(2) + 20, np(1) + 50, np(2) + 40]);
    add_line(sys, H("grid E").LConn(3), H("gE").LConn(1), "autorouting", "on");
    nm = ["E in", "E ph"];  un = ["V", "rad"];  tg = ["E_mag", "E_ph"];
    for j = 1:2
        add_block("nesl_utility/Simulink-PS Converter", sys + "/" + nm(j), "Unit", un(j), "Orientation", "left", ...
            "Position", [0 0 20 20]);
        pp = get_param(H("grid E").LConn(j), "Position");
        set_param(sys + "/" + nm(j), "Position", [pp(1) + 40, pp(2) - 10, pp(1) + 60, pp(2) + 10]);
        add_line(sys, H(nm(j)).RConn(1), H("grid E").LConn(j));
        f = addFrom(sys, tg(j), tagColor(cm, tg(j)));
        cp = get_param(sys + "/" + nm(j), "Position");  fpp = get_param(sys + "/" + f, "Position");
        set_param(sys + "/" + f, "Position", round([cp(3) + 30, (cp(2)+cp(4))/2 - 11, cp(3) + 30 + fpp(3) - fpp(1), (cp(2)+cp(4))/2 + 11]));
        add_line(sys, f + "/1", nm(j) + "/1");
        blks = [blks, nm(j), f]; %#ok<AGROW>
    end
    pq = ["P_poc", "Q_poc"];
    for j = 1:2
        g = addGoto(sys, pq(j), tagColor(cm, pq(j)));
        gp = get_param(sys + "/" + g, "Position");  mpos = get_param(sys + "/POC meter", "Position");
        yo = portY(sys + "/POC meter", "out", j);
        set_param(sys + "/" + g, "Position", [mpos(3) + 30, yo - 11, mpos(3) + 30 + gp(3) - gp(1), yo + 11]);
        add_line(sys, "POC meter/" + j, g + "/1");
        blks(end+1) = g; %#ok<AGROW>
    end
    blks = [blks, tapLabel(sys, "POC", H("POC meter").RConn(1)), "HPP transformer", "POC meter", "grid Zg", "grid E", "gE"];
    hGrid = areaAround(sys, "HPP transformer 33-132 kV, POC meter and grid (E behind Zg)", blks, areaColor("interface"));

    % (3) HPP bus and POC voltages, solver configuration
    gpos = get_param(hGrid, "Position");  yV = gpos(4) + 70;  blks = strings(0);
    vn = ["HPP bus voltage", "POC voltage"];  vb = ["p.net.Vhpp", "p.grid.Vpoc"];
    lab = ["HPP_bus", "POC"];  vt = ["V_bus", "V_poc"];
    for j = 1:2
        y0 = yV + 80*(j-1);
        vMeterBlock(sys, vn(j), vb(j), [xH + 160, y0, xH + 250, y0 + 40], lib);
        blks(end+1) = nodeLabel(sys, lab(j), H(vn(j)).LConn(1), "left"); %#ok<AGROW>
        g = addGoto(sys, vt(j), tagColor(cm, vt(j)));  gp = get_param(sys + "/" + g, "Position");
        yo = portY(sys + "/" + vn(j), "out", 1);
        set_param(sys + "/" + g, "Position", [xH + 290, yo - 11, xH + 290 + gp(3) - gp(1), yo + 11]);
        add_line(sys, vn(j) + "/1", g + "/1");
        blks = [blks, vn(j), g]; %#ok<AGROW>
    end
    add_block("nesl_utility/Solver Configuration", sys + "/solver", "EquationFormulation", "NE_FREQUENCY_TIME_EF", ...
        "UseLocalSolver", "on", "LocalSolverChoice", "NE_BACKWARD_EULER_ADVANCER", "LocalSolverSampleTime", "p.sim.Ts", ...
        "DoFixedCost", "on", "MaxNonlinIter", "1", "Position", [xH + 160, yV + 170, xH + 220, yV + 200]);
    blks = [blks, "solver", nodeLabel(sys, "HPP_bus", H("solver").RConn(1), "right")];
    hV = areaAround(sys, "Bus and POC voltages, solver configuration", blks, areaColor("interface"));
    ep = get_param(sys + "/grid E", "Position");

    % ================= outputs =================
    gpos = get_param(hGrid, "Position");  xo = gpos(3) + 300;
    add_block("simulink/Signal Routing/Mux", sys + "/V_re all", "Inputs", num2str(nA), "Position", [0 0 6 30*nA]);
    add_block("simulink/Signal Routing/Mux", sys + "/V_im all", "Inputs", num2str(nA), "Position", [0 0 6 30*nA]);
    add_block("simulink/Discrete/Memory", sys + "/V_re prev", "InitialCondition", "ones(" + nA + ",1)", "Position", [0 0 30 30]);
    add_block("simulink/Discrete/Memory", sys + "/V_im prev", "InitialCondition", "zeros(" + nA + ",1)", "Position", [0 0 30 30]);
    add_block("simulink/Signal Routing/Mux", sys + "/P_plant all", "Inputs", "3", "Position", [0 0 6 90]);
    add_block("simulink/Signal Routing/Mux", sys + "/Q_plant all", "Inputs", "3", "Position", [0 0 6 90]);
    add_block("simulink/Sources/In1", sys + "/f grid", "Port", "4", "Position", [0 0 30 14]);
    add_block("simulink/Signal Routing/Bus Creator", sys + "/network bus", "Inputs", "6", "Position", [0 0 6 300]);
    add_block("simulink/Signal Routing/Bus Creator", sys + "/POC bus", "Inputs", "3", "Position", [0 0 6 150]);
    add_block("simulink/Sinks/Out1", sys + "/network", "Port", "1", "Position", [0 0 30 14]);
    add_block("simulink/Sinks/Out1", sys + "/POC meas", "Port", "2", "Position", [0 0 30 14]);
    sre = [num2cell((2:nA)'), num2cell("@Vre_" + (2:nA)')];
    sim_ = [num2cell((2:nA)'), num2cell("@Vim_" + (2:nA)')];
    rows = {
        [stg("@Vre_1", 0), stg("V_re all", 1, sre), stg("V_re prev"), stg(">V_re_prev")]
        [stg("@Vim_1", 0), stg("V_im all", 1, sim_), stg("V_im prev"), stg(">V_im_prev")]
        [stg("@P_wpp", 0), stg("P_plant all", 1, {2, "@P_spp"; 3, "@P_bess"}), stg(">P_plant")]
        [stg("@Q_wpp", 0), stg("Q_plant all", 1, {2, "@Q_spp"; 3, "@Q_bess"}), stg(">Q_plant")]
        [stg("@V_re_prev", 0), stg("network bus", 1, {2, "@V_im_prev"; 3, "@P_plant"; 4, "@Q_plant"; 5, "@V_bus"; 6, "f grid"}), stg("network")]
        [stg("@P_poc", 0), stg("POC bus", 1, {2, "@Q_poc"; 3, "@V_poc"}), stg("POC meas")]};
    [~, hOut] = ctrlSection(sys, "Output buses (V one step old, powers in W and var)", rows, xo, 20, cm, areaColor("interface"));
    nameInputs(sys + "/network bus", ["V_re", "V_im", "P_plant", "Q_plant", "V_bus", "f"]);
    nameInputs(sys + "/POC bus", ["P_poc", "Q_poc", "V_poc"]);
    alignColumns(hIn, {hMid, [hDip, hGrid, hV]}, hOut);
    set_param(sys, "Description", "Network as a Simscape Electrical model in frequency-and-time mode " + ...
        "(buildNetworkSimscape). Same ports as the nodal network. Electrical nodes are joined by " + ...
        "Connection Labels (collector, tap, HPP bus, POC); signals by local Goto/From.");
    hideStructuralNames(sys);
end

function alignPhys(blk, x, yPort)
    % move a block so its left edge is at x and its first left physical
    % port (LConn 1) is at height yPort
    pos = get_param(blk, "Position");  w = pos(3) - pos(1);  h = pos(4) - pos(2);
    set_param(blk, "Position", [x, pos(2), x + w, pos(4)]);
    ph = get_param(blk, "PortHandles");  pp = get_param(ph.LConn(1), "Position");
    d = round(yPort - pp(2));
    set_param(blk, "Position", [x, pos(2) + d, x + w, pos(2) + d + h]);
end

function name = nodeLabel(sys, label, port, side)
    % Connection Label at a physical port: "right" puts it right of the port
    % (port on its left), "left" puts it left of the port (flipped)
    i = 1;
    while getSimulinkBlockHandle(sys + "/" + label + " label " + i) ~= -1, i = i + 1; end
    name = label + " label " + i;
    if side == "left", o = "left"; else, o = "right"; end
    add_block("nesl_utility/Connection Label", sys + "/" + name, "Label", label, "Orientation", o, "Position", [0 0 20 20]);
    pp = get_param(port, "Position");
    if side == "left"
        set_param(sys + "/" + name, "Position", [pp(1) - 60, pp(2) - 10, pp(1) - 40, pp(2) + 10]);
    else
        set_param(sys + "/" + name, "Position", [pp(1) + 40, pp(2) - 10, pp(1) + 60, pp(2) + 10]);
    end
    ph = get_param(sys + "/" + name, "PortHandles");
    add_line(sys, port, ph.LConn(1), "autorouting", "on");
end

function name = tapLabel(sys, label, port)
    % Connection Label below a physical port (a branch of that node)
    i = 1;
    while getSimulinkBlockHandle(sys + "/" + label + " label " + i) ~= -1, i = i + 1; end
    name = label + " label " + i;
    pp = get_param(port, "Position");
    add_block("nesl_utility/Connection Label", sys + "/" + name, "Label", label, "Orientation", "down", ...
        "Position", [pp(1) - 10, pp(2) + 40, pp(1) + 10, pp(2) + 60]);
    ph = get_param(sys + "/" + name, "PortHandles");
    add_line(sys, port, ph.LConn(1), "autorouting", "on");
end

function lib = eeLibrary()
    % lookup of ee_lib blocks by their path with spaces instead of line breaks
    raw = find_system('ee_lib', 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'BlockType', 'SimscapeBlock');
    all = string(strrep(raw, newline, ' '));
    lib = @(n) raw{all == "ee_lib/" + n};
end

function trBlock(sys, name, d, V1, V2, pos, lib)
    % two-winding Yg/Yg transformer: d = parameter struct path with S, uk, XR;
    % leakage and winding resistance split equally; magnetizing branch
    % from d.Rm, d.Xm (needed in frequency-and-time mode; same branch in the nodal model)
    r = "(" + d + ".uk/sqrt(1+" + d + ".XR^2))";
    add_block(lib("Passive/Transformers/Two-Winding Transformer (Three-Phase)"), sys + "/" + name, ...
        "SRated", d + ".S", "FRated", "p.grid.f0", "VRated1", V1, "VRated2", V2, ...
        "Winding1Connection", "ee.enum.windingconnection.Yg", "Winding2Connection", "ee.enum.windingconnection.Yg", ...
        "pu_Rw1", r + "/2", "pu_Rw2", r + "/2", "leakage_reactance_option", "ee.enum.transformer_leakage.include", ...
        "pu_Xl1", r + "*" + d + ".XR/2", "pu_Xl2", r + "*" + d + ".XR/2", ...
        "magnetizing_reactance_option", "ee.enum.transformer_magnetizingReactance.include", "pu_Xm", d + ".Xm", ...
        "magnetizing_resistance_option", "ee.enum.transformer_magnetizingResistance.include", "pu_Rm", d + ".Rm", ...
        "Position", pos);
    % run-time parameters: transformer data change without recompiling
    for c = ["SRated", "VRated1", "VRated2", "pu_Rw1", "pu_Rw2", "pu_Xl1", "pu_Xl2", "pu_Rm", "pu_Xm"]
        set_param(sys + "/" + name, c + "_conf", "runtime");
    end
end

function piBlock(sys, name, cab, len, f0, pos, lib)
    % feeder segment (subsystem): pi model from basic three-phase blocks,
    % series R-L and half the charging capacitance to ground at each end.
    % Port a (left) = far end (tap), b (right) = towards the busbar.
    % cab = cable parameter path (R, X [ohm/km], C [F/km], nPar); len [km].
    % (The library's Transmission Line (Three-Phase) block produced a
    % non-physical 100 Hz power ripple in frequency-and-time mode.)
    b = sys + "/" + name;
    add_block("built-in/Subsystem", b, "Position", pos);
    add_block("nesl_utility/Connection Port", b + "/a", "Side", "Left", "Position", [20 60 50 80]);
    add_block("nesl_utility/Connection Port", b + "/b", "Side", "Right", "Position", [330 60 360 80]);
    add_block(lib("Passive/RLC Assemblies/RLC (Three-Phase)"), b + "/series", "component_structure", ...
        "ee.enum.rlc.structure.SeriesRL", "R_conf", "runtime", "L_conf", "runtime", "R", cab + ".R*" + len + "/" + cab + ".nPar", ...
        "L", cab + ".X*" + len + "/(2*pi*" + f0 + ")/" + cab + ".nPar", "Position", [160 50 220 90]);
    for e = ["a", "b"]
        x = 80 + 180*(e == "b");
        add_block(lib("Passive/RLC Assemblies/RLC (Three-Phase)"), b + "/C" + e, "component_structure", ...
            "ee.enum.rlc.structure.C", "C_conf", "runtime", "C", cab + ".C*" + len + "*" + cab + ".nPar/2", "Position", [x 130 x+40 170]);
        add_block(lib("Connectors & References/Grounded Neutral (Three-Phase)"), b + "/gnd " + e, "Position", [x 200 x+30 215]);
    end
    h = @(x) get_param(b + "/" + x, "PortHandles");
    add_line(b, h("a").RConn(1), h("series").LConn(1));  add_line(b, h("b").RConn(1), h("series").RConn(1));
    add_line(b, h("Ca").LConn(1), h("series").LConn(1));  add_line(b, h("Cb").LConn(1), h("series").RConn(1));
    add_line(b, h("Ca").RConn(1), h("gnd a").LConn(1));  add_line(b, h("Cb").RConn(1), h("gnd b").LConn(1));
    set_param(b, "BackgroundColor", "[0.95 0.95 0.95]");
end

function meterBlock(sys, name, pos, lib)
    % subsystem: three-phase power sensor between ports a (left) and b
    % (right); outputs P [W], Q [var] flowing from a to b
    b = sys + "/" + name;
    add_block("built-in/Subsystem", b, "Position", pos);
    add_block(lib("Sensors & Transducers/Power Sensor (Three-Phase)"), b + "/PQ", "f0", "p.grid.f0", "Position", [200 100 260 160]);
    add_block("nesl_utility/Connection Port", b + "/a", "Side", "Left", "Position", [100 120 130 140]);
    add_block("nesl_utility/Connection Port", b + "/b", "Side", "Right", "Position", [330 120 360 140]);
    for j = 1:2
        n = ["P", "Q"];
        add_block("nesl_utility/PS-Simulink Converter", b + "/" + n(j) + " out", "Position", [200+60*(j-1) 30 220+60*(j-1) 50]);
        add_block("simulink/Sinks/Out1", b + "/" + n(j), "Port", num2str(j), "Position", [200+60*(j-1) -20 230+60*(j-1) -6]);
    end
    h = @(x) get_param(b + "/" + x, "PortHandles");
    add_line(b, h("a").RConn(1), h("PQ").LConn(1));  add_line(b, h("b").RConn(1), h("PQ").RConn(1));
    add_line(b, h("PQ").LConn(2), h("P out").LConn(1));  add_line(b, h("PQ").LConn(3), h("Q out").LConn(1));
    add_line(b, "P out/1", "P/1");  add_line(b, "Q out/1", "Q/1");
    set_param(b, "BackgroundColor", "[0.95 0.95 0.95]");
end

function vMeterBlock(sys, name, Vn, pos, lib)
    % subsystem: phase voltages at the port -> rms line voltage [pu of Vn]
    b = sys + "/" + name;
    add_block("built-in/Subsystem", b, "Position", pos);
    add_block("nesl_utility/Connection Port", b + "/a", "Side", "Left", "Position", [20 60 50 80]);
    add_block(lib("Sensors & Transducers/Phase Voltage Sensor (Three-Phase)"), b + "/Vs", "Position", [100 50 140 90]);
    add_block(lib("Connectors & References/Grounded Neutral (Three-Phase)"), b + "/gn", "Position", [100 130 130 145]);
    add_block("nesl_utility/PS-Simulink Converter", b + "/v", "Position", [180 60 200 80]);
    add_block("simulink/Sources/Digital Clock", b + "/clock", "SampleTime", "p.sim.Ts", "Position", [180 110 200 130]);
    addPhasor(b, [240 50 340 130], Vn);
    add_block("simulink/Math Operations/Math Function", b + "/abs", "Operator", "hypot", "Position", [380 60 420 100]);
    add_block("simulink/Sinks/Out1", b + "/V", "Port", "1", "Position", [460 75 490 89]);
    h = @(x) get_param(b + "/" + x, "PortHandles");
    add_line(b, h("a").RConn(1), h("Vs").LConn(1));  add_line(b, h("Vs").RConn(2), h("gn").LConn(1));
    add_line(b, h("Vs").RConn(1), h("v").LConn(1));
    add_line(b, "v/1", "phasor/1");  add_line(b, "clock/1", "phasor/2");
    add_line(b, "phasor/1", "abs/1");  add_line(b, "phasor/2", "abs/2");  add_line(b, "abs/1", "V/1");
    set_param(b, "BackgroundColor", "[0.95 0.95 0.95]");
end

function convBlock(sys, name, Ib, Vn, pos, lib)
    % converter interface (subsystem): in 1 I_re, 2 I_im [system pu, network
    % angle reference], 3 time; out 1 V_re, 2 V_im [pu] at the terminal;
    % physical port ~ (right). Controlled three-phase current source
    % (magnitude rms, phase) and a phase voltage sensor.
    b = sys + "/" + name;
    add_block("built-in/Subsystem", b, "Position", pos);
    add_block("simulink/Sources/In1", b + "/I_re", "Port", "1", "Position", [20 40 50 54]);
    add_block("simulink/Sources/In1", b + "/I_im", "Port", "2", "Position", [20 80 50 94]);
    add_block("simulink/Sources/In1", b + "/t", "Port", "3", "Position", [20 200 50 214]);
    body = [
        "% current phasor (system pu, network angle) -> rms magnitude [A] and phase [rad]"
        "I = complex(I_re, I_im);"
        "Imag = abs(I) * Ib;"
        "Iph = angle(I);"];
    addMatlabFunction(b + "/current", "current", ["I_re", "I_im", "Ib"], ["Imag", "Iph"], body, "", [120 30 220 110]);
    add_block("simulink/Sources/Constant", b + "/Ib", "Value", Ib, "Position", [40 120 100 140]);
    add_block("nesl_utility/Simulink-PS Converter", b + "/I in", "Unit", "A", "Position", [260 40 280 60]);
    add_block("nesl_utility/Simulink-PS Converter", b + "/ph in", "Unit", "rad", "Position", [260 80 280 100]);
    add_block(lib("Sources/Controlled Current Source (Three-Phase)"), b + "/source", "input_option", ...
        "ee.enum.inputType.singleFreqAC", "FRated", "p.grid.f0", "Position", [320 30 400 110]);
    add_block(sprintf("ee_lib/Connectors &\nReferences/Electrical Reference"), b + "/gnd", "Position", [330 140 350 160]);
    add_block("nesl_utility/Connection Port", b + "/~", "Side", "Right", "Position", [500 60 530 80]);
    add_block(lib("Sensors & Transducers/Phase Voltage Sensor (Three-Phase)"), b + "/Vs", "Position", [440 150 480 190]);
    add_block(lib("Connectors & References/Grounded Neutral (Three-Phase)"), b + "/gn", "Position", [440 220 470 235]);
    add_block("nesl_utility/PS-Simulink Converter", b + "/v", "Position", [380 160 400 180]);
    addPhasor(b, [240 150 340 230], Vn);
    add_block("simulink/Sinks/Out1", b + "/V_re", "Port", "1", "Position", [560 160 590 174]);
    add_block("simulink/Sinks/Out1", b + "/V_im", "Port", "2", "Position", [560 200 590 214]);
    h = @(x) get_param(b + "/" + x, "PortHandles");
    add_line(b, "I_re/1", "current/1");  add_line(b, "I_im/1", "current/2");  add_line(b, "Ib/1", "current/3");
    add_line(b, "current/1", "I in/1");  add_line(b, "current/2", "ph in/1");
    add_line(b, h("I in").RConn(1), h("source").LConn(1));  add_line(b, h("ph in").RConn(1), h("source").LConn(2));
    add_line(b, h("source").LConn(3), h("gnd").LConn(1));
    add_line(b, h("source").RConn(1), h("~").RConn(1));
    add_line(b, h("Vs").LConn(1), h("source").RConn(1));  add_line(b, h("Vs").RConn(2), h("gn").LConn(1));
    add_line(b, h("Vs").RConn(1), h("v").LConn(1));
    add_line(b, "v/1", "phasor/1");  add_line(b, "t/1", "phasor/2");
    add_line(b, "phasor/1", "V_re/1");  add_line(b, "phasor/2", "V_im/1");
    set_param(b, "BackgroundColor", "[0.878 0.965 0.925]");
end

function body = phasorBody()
    % space phasor of the phase voltages at the fixed nominal frequency w0,
    % rotated by +90 deg so the grid source is at angle 0 (sine reference
    % of the library), scaled to pu of the nominal line voltage Vn
    body = [
        "% space phasor at the nominal frequency: exact phasor in frequency-and-time mode"
        "a = complex(-0.5, sqrt(3)/2);"
        "s = 2/3*(v(1) + a*v(2) + a^2*v(3)) * exp(complex(0, pi/2 - w0*t));"
        "V = s * sqrt(3/2) / Vn;"
        "V_re = real(V);"
        "V_im = imag(V);"];
end

function addPhasor(b, pos, Vn)
    % MATLAB Function "phasor" (inputs v, t, Vn, w0) with constants for Vn and w0
    addMatlabFunction(b + "/phasor", "phasor", ["v", "t", "Vn", "w0"], ["V_re", "V_im"], phasorBody(), "", pos);
    add_block("simulink/Sources/Constant", b + "/Vn", "Value", Vn, "Position", [pos(1)-90 pos(4)+10 pos(1)-30 pos(4)+30]);
    add_block("simulink/Sources/Constant", b + "/w0", "Value", "2*pi*p.grid.f0", "Position", [pos(1)-90 pos(4)+40 pos(1)-30 pos(4)+60]);
    add_line(b, "Vn/1", "phasor/3");  add_line(b, "w0/1", "phasor/4");
end
