function buildHPP(opts)
%BUILDHPP  (Re)build the hierarchical HPP model from the parameters.
%
%   buildHPP()                    rebuild the top model HPP.slx for the
%                                 asset counts N in HPP_parameters()
%   buildHPP(All=true)            also rebuild every controller and asset
%                                 model (overwrites them: placeholder logic)
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
        specs = controllerSpecs();
        for k = 1:numel(specs)
            buildController(specs(k), dirs.ctrl);
        end
        buildAsset("WT",   dirs.assets);
        buildAsset("PV",   dirs.assets);
        buildAsset("BESS", dirs.assets);
    end

    buildTop(p, dirs.top);
end

%% ===================================================================
%  Controller definitions (stage 1: placeholder logic)
%  ===================================================================
function specs = controllerSpecs()
    specs = struct("name", {}, "cfield", {}, "inputs", {}, "outputs", {}, "code", {}, "doc", {});

    % ---- HPP controller (base: POC) ----
    in = ["P_ref","Q_ref","f_meas","P_poc","Q_poc","V_poc", ...
          "P_avail_wpp","P_avail_spp","P_chg_max_bess","P_dis_max_bess","SOC_bess", ...
          "P_wpp","Q_wpp","P_spp","Q_spp","P_bess","Q_bess"];
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
%  Controller model builder
%  ===================================================================
function buildController(s, folder)
    mdl = char(s.name);
    closeIfLoaded(mdl);
    new_system(mdl);
    set_param(mdl, "SolverType", "Fixed-step", "Solver", "FixedStepDiscrete", ...
        "FixedStep", "Ts", "StopTime", "inf", "Description", s.doc);

    usesN = any(string({s.inputs.dims}) == "N");
    mdl = string(mdl);
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
    h = 40 * max(nIn, nOut) + 40;
    addMatlabFunction(mdl + "/logic", "logic", inNames, outNames, s.code, "par", [300 30 520 30+h]);

    for k = 1:nIn
        add_block("simulink/Sources/In1", mdl + "/" + inNames(k), "Port", num2str(k), ...
            "OutDataTypeStr", "double", "PortDimensions", inDims(k), "SampleTime", "Ts", ...
            "Position", [60, 40*k, 90, 40*k+14]);
        add_line(mdl, inNames(k) + "/1", "logic/" + k, "autorouting", "smart");
    end
    for k = 1:nOut
        add_block("simulink/Sinks/Out1", mdl + "/" + outNames(k), "Port", num2str(k), ...
            "OutDataTypeStr", "double", "Position", [700, 40*k, 730, 40*k+14]);
        add_line(mdl, "logic/" + k, outNames(k) + "/1", "autorouting", "smart");
    end
    save_system(mdl, fullfile(folder, mdl + ".slx"));
    close_system(mdl);
    registerInProject(fullfile(folder, mdl + ".slx"));
end

%% ===================================================================
%  Asset model builder (plant side, not a DLL)
%  ===================================================================
function buildAsset(type, folder)
    mdl = char(type + "_asset");
    ctrl = char(type + "_ctrl");
    t = lower(type);
    closeIfLoaded(mdl);
    new_system(mdl);
    set_param(mdl, "SolverType", "Fixed-step", "Solver", "ode4", "FixedStep", "Tsim", "StopTime", "inf", ...
        "Description", type + " asset: " + ctrl + " (asset controller) + placeholder generator model " + ...
                       "(current source aligned with the terminal voltage). Per unit on the asset rating " + ...
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

    % inports: set-points, terminal voltage phasor (from the network, one
    % step old), primary source
    ins = ["P_set", "Q_set", "V_re", "V_im"];
    if type == "WT", ins(end+1) = "P_wind_avail"; end
    if type == "PV", ins(end+1) = "P_pv_avail"; end
    for k = 1:numel(ins)
        add_block("simulink/Sources/In1", mdl + "/" + ins(k), "Port", num2str(k), ...
            "OutDataTypeStr", "double", "PortDimensions", "1", "Position", [40, 50*k, 70, 50*k+14]);
    end
    L = @(a, b) add_line(mdl, a, b, "autorouting", "smart");

    % terminal voltage magnitude for the controller
    addMatlabFunction(mdl + "/terminal", "terminal", ["V_re", "V_im"], "V_term", ...
        "V_term = sqrt(V_re^2 + V_im^2);", "", [130 140 200 200]);
    L("V_re/1", "terminal/1"); L("V_im/1", "terminal/2");

    % asset controller instance
    cb = mdl + "/ctrl";
    add_block("simulink/Ports & Subsystems/Model", cb, "ModelName", ctrl, "Position", [240 40 400 340]);
    Simulink.ModelReference.refresh(cb);
    setArg(cb, "par", "ap.ctrl");
    L("P_set/1", "ctrl/1"); L("Q_set/1", "ctrl/2"); L("terminal/1", "ctrl/3");

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
    addMatlabFunction(mdl + "/generator", "generator", ["ip_cmd", "iq_cmd", "V_re", "V_im"], ...
        ["P", "Q", "I_re", "I_im"], gen, "", [480 40 600 200]);
    L("ctrl/1", "generator/1"); L("ctrl/2", "generator/2");
    L("V_re/1", "generator/3"); L("V_im/1", "generator/4");

    % measured P, Q back to the controller (one step old)
    add_block("simulink/Discrete/Memory", mdl + "/P meas", "Position", [480 260 510 290]);
    add_block("simulink/Discrete/Memory", mdl + "/Q meas", "Position", [480 320 510 350]);
    L("generator/1", "P meas/1"); L("generator/2", "Q meas/1");
    L("P meas/1", "ctrl/4"); L("Q meas/1", "ctrl/5");

    % outports: 1 P, 2 Q, 3 P_avail, 4 I_re, 5 I_im (asset pu), 6 SOC (BESS)
    outs = ["P", "Q", "P_avail", "I_re", "I_im"];
    for k = 1:numel(outs)
        add_block("simulink/Sinks/Out1", mdl + "/" + outs(k), "Port", num2str(k), ...
            "Position", [700, 50*k, 730, 50*k+14]);
    end
    L("generator/1", "P/1"); L("generator/2", "Q/1"); L("ctrl/3", "P_avail/1");
    L("generator/3", "I_re/1"); L("generator/4", "I_im/1");

    switch type
        case "WT", L("P_wind_avail/1", "ctrl/6");
        case "PV", L("P_pv_avail/1", "ctrl/6");
        case "BESS"
            % battery energy: dSOC/dt = -kSOC * P  (P in pu, + = discharge)
            add_block("simulink/Math Operations/Gain", mdl + "/kSOC", "Gain", "-ap.kSOC", "Position", [560 400 600 430]);
            add_block("simulink/Continuous/Integrator", mdl + "/SOC int", "InitialCondition", "ap.SOC0", ...
                "Position", [640 400 670 430]);
            add_block("simulink/Sinks/Out1", mdl + "/SOC", "Port", "6", "Position", [760 408 790 422]);
            L("generator/1", "kSOC/1"); L("kSOC/1", "SOC int/1"); L("SOC int/1", "SOC/1"); L("SOC int/1", "ctrl/6");
    end
    save_system(mdl, fullfile(folder, mdl + ".slx"));
    close_system(mdl);
    registerInProject(fullfile(folder, mdl + ".slx"));
end

%% ===================================================================
%  Top model builder
%  ===================================================================
function buildTop(p, folder)
    m = 'HPP';
    closeIfLoaded(m);
    ws0 = warning("off", "Simulink:Engine:MdlFileShadowing");   % the old HPP.slx is replaced below
    restoreWarn = onCleanup(@() warning(ws0));
    new_system(m);
    set_param(m, "SolverType", "Fixed-step", "Solver", "ode4", "FixedStep", "p.sim.Ts", ...
        "StopTime", "p.sim.Tstop", "SaveOutput", "on", "SignalLogging", "on", ...
        "AlgebraicLoopMsg", "error", ...
        "Description", "Hierarchical HPP model, generated by utilities/buildHPP.m. Do not edit by hand: rebuild instead.");
    T = topTools(m);

    % ---------------- HPPC ----------------
    T.model("HPPC", "HPPC", [600 40 760 760]);
    setArg(m + "/HPPC", "par", "p.ctrl.hppc.par");
    hb = "p.ctrl.hppc.Pbase";

    % TSO set-points and grid signals (stage 1: constants)
    T.const("P_ref", "p.grid.Pref", [30 50]);
    % TSO delays start at the initial set-point (not 0), so the HPPC does not
    % see a zero reference during the first p.comm.tso seconds
    T.chain("P_ref/1", ["gain:1/" + hb, "delay:p.comm.tso@p.grid.Pref/" + hb], "HPPC/1", "P_ref");
    T.const("Q_ref", "0", [30 110]);
    T.chain("Q_ref/1", "delay:p.comm.tso@0", "HPPC/2", "Q_ref");
    T.const("f_meas", "1", [30 170]);         T.line("f_meas/1", "HPPC/3");

    % ---------------- network ----------------
    % system base p.grid.Pbase; "sb" converts system pu to the HPPC base
    sb = "p.grid.Pbase";
    nA = p.wt.N + p.pv.N + p.bess.N;
    buildNetwork(T.m + "/Network", [2100 900 2300 1300]);
    T.block("simulink/Signal Routing/Mux", "I_re all", [2000 900 2005 1000], "Inputs", "3");
    T.block("simulink/Signal Routing/Mux", "I_im all", [2000 1020 2005 1120], "Inputs", "3");
    T.line("I_re all/1", "Network/1");
    T.line("I_im all/1", "Network/2");
    % terminal voltages back to the assets, one step old (the generators'
    % "PLL"; also avoids an algebraic loop through the network)
    T.block("simulink/Discrete/Memory", "V_re prev", [2400 900 2430 930], "InitialCondition", sprintf("ones(%d,1)", nA));
    T.block("simulink/Discrete/Memory", "V_im prev", [2400 960 2430 990], "InitialCondition", sprintf("zeros(%d,1)", nA));
    T.line("Network/1", "V_re prev/1");
    T.line("Network/2", "V_im prev/1");
    T.block("simulink/Signal Routing/Demux", "V_re asset", [2480 900 2485 1000], "Outputs", num2str(nA));
    T.block("simulink/Signal Routing/Demux", "V_im asset", [2480 1020 2485 1120], "Outputs", num2str(nA));
    T.line("V_re prev/1", "V_re asset/1");
    T.line("V_im prev/1", "V_im asset/1");
    T.block("simulink/Signal Routing/Demux", "P_plant", [2400 1060 2405 1160], "Outputs", "3");
    T.block("simulink/Signal Routing/Demux", "Q_plant", [2400 1180 2405 1280], "Outputs", "3");
    T.line("Network/3", "P_plant/1");
    T.line("Network/4", "Q_plant/1");

    % POC measurements -> HPPC
    T.chain("Network/6", ["gain:" + sb + "/" + hb, "memory", "delay:p.comm.meas_hppc"], "HPPC/4", "P_poc meas");
    T.chain("Network/7", ["gain:" + sb + "/" + hb, "memory", "delay:p.comm.meas_hppc"], "HPPC/5", "Q_poc meas");
    T.chain("Network/8", ["memory", "delay:p.comm.meas_hppc@1"], "HPPC/6", "V_poc meas");

    % ---------------- plants ----------------
    plants = [ ...
        struct("tag","wpp",  "ctrl","WPPC",  "c","wppc",  "type","WT",   "t","wt",   "y",  40, "hppIn",[7 12 13],   "hppOut",[1 2]), ...
        struct("tag","spp",  "ctrl","SPPC",  "c","sppc",  "type","PV",   "t","pv",   "y", 900, "hppIn",[8 14 15],   "hppOut",[3 4]), ...
        struct("tag","bess", "ctrl","BESSC", "c","bessc", "type","BESS", "t","bess", "y",1760, "hppIn",[0 16 17],  "hppOut",[5 6])];
    offset = 0;
    for i = 1:numel(plants)
        buildPlant(T, p, plants(i), hb, sb, i, offset);
        offset = offset + p.(plants(i).t).N;
    end

    % ---------------- outports (logged as out.yout, W / var / pu) ----------------
    T.block("simulink/Math Operations/Gain", "P_poc to W", [2400 1320 2460 1350], "Gain", sb);
    T.block("simulink/Math Operations/Gain", "Q_poc to var", [2400 1380 2460 1410], "Gain", sb);
    T.line("Network/6", "P_poc to W/1");
    T.line("Network/7", "Q_poc to var/1");
    T.outport("P_poc", "P_poc to W/1", 1);
    T.outport("SOC", "SOC plant/1", 2);
    T.block("simulink/Math Operations/Gain", "P_bess_set to W", [900 600 960 630], "Gain", hb);
    T.line("HPPC/5", "P_bess_set to W/1");
    T.outport("P_bess_set", "P_bess_set to W/1", 3);
    T.outport("Q_poc", "Q_poc to var/1", 4);
    for i = 1:3
        g = "P_" + plants(i).tag + " to W";
        T.block("simulink/Math Operations/Gain", g, [2500 1060+40*i 2560 1080+40*i], "Gain", sb);
        T.line(sprintf("P_plant/%d", i), g + "/1");
        T.outport("P_" + plants(i).tag, g + "/1", 4 + i);
    end
    for i = 1:3
        g = "Q_" + plants(i).tag + " to var";
        T.block("simulink/Math Operations/Gain", g, [2500 1220+40*i 2560 1240+40*i], "Gain", sb);
        T.line(sprintf("Q_plant/%d", i), g + "/1");
        T.outport("Q_" + plants(i).tag, g + "/1", 7 + i);
    end
    T.outport("V_poc", "Network/8", 11);
    T.outport("V_bus", "Network/5", 12);

    % record what this build was made for; runCase/runSweep check it
    ws = get_param(m, "ModelWorkspace");
    assignin(ws, "buildInfo", struct("N_wt", p.wt.N, "N_pv", p.pv.N, "N_bess", p.bess.N, ...
        "built", string(datetime("now"))));
    save_system(m, fullfile(folder, "HPP.slx"));
    fprintf("Built HPP.slx with N = %d wind, %d PV, %d BESS asset instances.\n", p.wt.N, p.pv.N, p.bess.N);
end

function buildPlant(T, p, pl, hb, sb, idx, offset)
    % idx: plant number (1 wind, 2 PV, 3 BESS) in the network outputs;
    % offset: global index of this plant's first asset in the network
    N = p.(pl.t).N;
    y = pl.y;
    pb = "p.ctrl." + pl.c + ".Pbase";
    ab = "p.ctrl." + pl.t + ".Pbase";
    C = pl.ctrl;
    isB = pl.type == "BESS";

    % plant controller
    T.model(C, C, [1000 y 1160 y+420]);
    setArg(T.m + "/" + C, "par", "p.ctrl." + pl.c + ".par");

    % HPPC -> plant controller (P_set, Q_set)
    T.chain(sprintf("HPPC/%d", pl.hppOut(1)), ["delay:p.comm.hppc_to_plant", "gain:" + hb + "/" + pb], C + "/1", "P_set_" + pl.tag);
    T.chain(sprintf("HPPC/%d", pl.hppOut(2)), ["delay:p.comm.hppc_to_plant", "gain:" + hb + "/" + pb], C + "/2", "Q_set_" + pl.tag);

    % asset instances
    T.block("simulink/Signal Routing/Demux", C + " P_set_k", [1400 y 1405 y+120], "Outputs", num2str(N));
    T.block("simulink/Signal Routing/Demux", C + " Q_set_k", [1400 y+140 1405 y+260], "Outputs", num2str(N));
    T.chain(C + "/1", ["delay:p.comm.plant_to_asset", "gain:" + pb + "/" + ab], C + " P_set_k/1", "P_set_k " + pl.tag);
    T.chain(C + "/2", ["delay:p.comm.plant_to_asset", "gain:" + pb + "/" + ab], C + " Q_set_k/1", "Q_set_k " + pl.tag);
    % asset outputs: 1 P, 2 Q, 3 P_avail, 4 I_re, 5 I_im, 6 SOC (BESS)
    muxNames = ["P", "Q", "P_avail", "I_re", "I_im", "SOC"];
    outIdx = 1:5;
    if isB, outIdx = 1:6; end
    for j = outIdx
        T.block("simulink/Signal Routing/Mux", pl.tag + " " + muxNames(j) + "_k", ...
            [1720 y+60*(j-1) 1725 y+60*(j-1)+50], "Inputs", num2str(N));
    end
    for k = 1:N
        a = sprintf("%s %d", pl.type, k);
        g = offset + k;                       % global asset index in the network
        ya = y + 300 + 130*(k-1);
        T.model(a, pl.type + "_asset", [1480 ya 1620 ya+110]);
        setArg(T.m + "/" + a, "ap", sprintf("p.%s.ap(%d)", pl.t, k));
        T.line(sprintf("%s P_set_k/%d", C, k), a + "/1");
        T.line(sprintf("%s Q_set_k/%d", C, k), a + "/2");
        T.line(sprintf("V_re asset/%d", g), a + "/3");
        T.line(sprintf("V_im asset/%d", g), a + "/4");
        switch pl.type
            case "WT"
                src = sprintf("wind %d", k);
                T.block("simulink/Sources/Step", src, [1300 ya+70 1330 ya+100], "Time", "p.wt.tStep", ...
                    "Before", "p.wt.Pavail0", "After", "p.wt.PavailStep", "SampleTime", "0");
                T.line(src + "/1", a + "/5");
            case "PV"
                src = sprintf("irradiance %d", k);
                T.const(src, "p.pv.Pavail", [1300 ya+70]);
                T.line(src + "/1", a + "/5");
        end
        for j = outIdx
            T.line(sprintf("%s/%d", a, j), sprintf("%s %s_k/%d", pl.tag, muxNames(j), k));
        end
    end

    % asset P and Q (asset pu) are not used yet: terminate
    T.block("simulink/Sinks/Terminator", pl.tag + " P_k unused", [1780 y 1800 y+20]);
    T.block("simulink/Sinks/Terminator", pl.tag + " Q_k unused", [1780 y+60 1800 y+80]);
    T.line(pl.tag + " P_k/1", pl.tag + " P_k unused/1");
    T.line(pl.tag + " Q_k/1", pl.tag + " Q_k unused/1");

    % current injections (asset pu -> system pu) into the network
    T.chain(pl.tag + " I_re_k/1", "gain:" + ab + "/" + sb, sprintf("I_re all/%d", idx), "I_re " + pl.tag);
    T.chain(pl.tag + " I_im_k/1", "gain:" + ab + "/" + sb, sprintf("I_im all/%d", idx), "I_im " + pl.tag);

    % measurements at the plant's busbar connection -> plant controller
    T.chain(sprintf("P_plant/%d", idx), ["gain:" + sb + "/" + pb, "memory", "delay:p.comm.meas_plant"], C + "/3", "P_meas " + pl.tag);
    T.chain(sprintf("Q_plant/%d", idx), ["gain:" + sb + "/" + pb, "memory", "delay:p.comm.meas_plant"], C + "/4", "Q_meas " + pl.tag);
    T.chain("Network/5", ["memory", "delay:p.comm.meas_plant@1"], C + "/5", "V_meas " + pl.tag);
    T.chain(pl.tag + " P_avail_k/1", ["gain:" + ab + "/" + pb, "memory", "delay:p.comm.meas_plant"], C + "/6", "P_avail_k " + pl.tag);
    if isB
        T.chain(pl.tag + " SOC_k/1", ["memory", "delay:p.comm.meas_plant"], C + "/7", "SOC_k");
        % plant SOC for logging: share-weighted mean of the asset SOCs
        T.block("simulink/Math Operations/Gain", "SOC plant", [1800 y+400 1860 y+430], ...
            "Gain", "p.ctrl.bessc.par.share'", "Multiplication", "Matrix(K*u)");
        T.line(pl.tag + " SOC_k/1", "SOC plant/1");
    end

    % plant controller -> HPPC: P_avail, P_plant, Q_plant (BESS: limits and SOC
    % instead of P_avail)
    if isB
        T.block("simulink/Sinks/Terminator", "BESSC P_avail unused", [1200 y+120 1220 y+140]);
        T.line(C + "/3", "BESSC P_avail unused/1");
        T.chain(C + "/6", ["gain:" + pb + "/" + hb, "memory", "delay:p.comm.meas_hppc"], "HPPC/9", "P_chg_max bess");
        T.chain(C + "/7", ["gain:" + pb + "/" + hb, "memory", "delay:p.comm.meas_hppc"], "HPPC/10", "P_dis_max bess");
        T.chain(C + "/8", ["memory", "delay:p.comm.meas_hppc"], "HPPC/11", "SOC bess");
    else
        T.chain(C + "/3", ["gain:" + pb + "/" + hb, "memory", "delay:p.comm.meas_hppc"], sprintf("HPPC/%d", pl.hppIn(1)), "P_avail " + pl.tag);
    end
    T.chain(C + "/4", ["gain:" + pb + "/" + hb, "memory", "delay:p.comm.meas_hppc"], sprintf("HPPC/%d", pl.hppIn(2)), "P " + pl.tag);
    T.chain(C + "/5", ["gain:" + pb + "/" + hb, "memory", "delay:p.comm.meas_hppc"], sprintf("HPPC/%d", pl.hppIn(3)), "Q " + pl.tag);
end

%% ===================================================================
%  Small helpers for building the top model
%  ===================================================================
function T = topTools(m)
    T.m = string(m);
    n = 0;
    function name = uniq(base)
        n = n + 1;
        name = sprintf("%s #%d", base, n);
    end
    T.block = @(lib, name, pos, varargin) add_block(lib, m + "/" + name, "Position", pos, varargin{:});
    T.line  = @(a, b) add_line(m, a, b, "autorouting", "smart");
    T.const = @(name, val, xy) add_block("simulink/Sources/Constant", m + "/" + name, ...
        "Value", val, "Position", [xy(1) xy(2) xy(1)+60 xy(2)+30]);
    T.model = @(name, ref, pos) addModel(m, name, ref, pos);
    T.chain = @(src, steps, dst, label) chain(m, src, steps, dst, label, @uniq);
    T.outport = @(name, src, port) outport(m, name, src, port);
end

function addModel(m, name, ref, pos)
    b = m + "/" + name;
    add_block("simulink/Ports & Subsystems/Model", b, "ModelName", ref, "Position", pos);
    Simulink.ModelReference.refresh(b);
end

function chain(m, src, steps, dst, label, uniq)
    % src -> step1 -> step2 -> ... -> dst; steps: "gain:<expr>", "delay:<time>[@<init>]", "memory"
    prev = src;
    pp = get_param(m + "/" + extractBefore(src, "/"), "Position");
    x = pp(3) + 30; y = pp(2);
    first = true;
    for s = steps
        kind = extractBefore(s + ":", ":");
        arg  = extractAfter(s, ":");
        name = uniq(label + " " + kind);
        switch kind
            case "gain",   add_block("simulink/Math Operations/Gain", m + "/" + name, "Gain", arg, "Position", [x y x+50 y+24]);
            case "delay"   % "delay:<time>" or "delay:<time>@<initial output>"
                parts = split(arg, "@");
                init = "0";
                if numel(parts) > 1, init = parts(2); end
                add_block("simulink/Continuous/Transport Delay", m + "/" + name, "DelayTime", parts(1), ...
                    "InitialOutput", init, "Position", [x y x+50 y+24]);
            case "memory", add_block("simulink/Discrete/Memory", m + "/" + name, "Position", [x y x+30 y+24]);
        end
        h = add_line(m, prev, name + "/1", "autorouting", "smart");
        if first, set_param(h, "Name", label); first = false; end
        prev = name + "/1";
        x = x + 70;
    end
    add_line(m, prev, dst, "autorouting", "smart");
end

function buildNetwork(sub, pos)
    % Masked subsystem around one MATLAB Function block that solves the
    % phasor network each step. Mask parameter "net" = p.net.solver
    % (see networkSolverData). Inputs: asset current injections (system pu).
    % Outputs: 1 V_re, 2 V_im (asset terminals), 3 P_plant, 4 Q_plant (3x1,
    % delivered into the 33 kV busbar per plant), 5 V_bus, 6 P_poc, 7 Q_poc
    % (into the grid), 8 V_poc. All system pu.
    add_block("simulink/Ports & Subsystems/Subsystem", sub, "Position", pos);
    delete_line(sub, "In1/1", "Out1/1");
    delete_block(sub + "/In1"); delete_block(sub + "/Out1");
    ins  = ["I_re", "I_im"];
    outs = ["V_re", "V_im", "P_plant", "Q_plant", "V_bus", "P_poc", "Q_poc", "V_poc"];
    body = [
        "Z  = complex(net.Zre, net.Zim);"
        "nA = numel(I_re);"
        "n  = size(Z, 1);"
        "Zg = complex(net.Zg_re, net.Zg_im);"
        "I = complex(zeros(n, 1), zeros(n, 1));"
        "I(1:nA) = complex(I_re(:), I_im(:));"
        "I(n) = I(n) + net.E / Zg;                 % grid Norton source at the POC"
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
        "Spoc = Vp * conj((Vp - net.E) / Zg);      % power into the grid"
        "P_poc = real(Spoc);  Q_poc = imag(Spoc);  V_poc = abs(Vp);"];
    addMatlabFunction(sub + "/solve", "solve", ins, outs, body, "net", [200 40 380 360]);
    for k = 1:2
        add_block("simulink/Sources/In1", sub + "/" + ins(k), "Port", num2str(k), "Position", [60 60*k 90 60*k+14]);
        add_line(sub, ins(k) + "/1", "solve/" + k, "autorouting", "smart");
    end
    for k = 1:numel(outs)
        add_block("simulink/Sinks/Out1", sub + "/" + outs(k), "Port", num2str(k), "Position", [480 40*k 510 40*k+14]);
        add_line(sub, "solve/" + k, outs(k) + "/1", "autorouting", "smart");
    end
    mk = Simulink.Mask.create(sub);
    mk.addParameter("Name", "net", "Prompt", "Network solver data", "Value", "p.net.solver", ...
        "Evaluate", "on", "Tunable", "off");
    mk.Description = "Positive-sequence phasor network (collector system, transformers, grid). " + ...
        "Solves V = Z*I each step. Data from networkSolverData(p).";
end

function outport(m, name, src, port)
    b = m + "/" + name;
    add_block("simulink/Sinks/Out1", b, "Port", num2str(port), "Position", [2100 60*port 2130 60*port+14]);
    h = add_line(m, src, name + "/1", "autorouting", "smart");
    set_param(h, "Name", name);
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
