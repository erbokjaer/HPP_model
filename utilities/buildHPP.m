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
        "% Stage 1 placeholder: wind and PV deliver what is available, the BESS"
        "% covers the rest of P_ref within its limits. No Q control yet."
        "P_set_wpp  = P_avail_wpp;"
        "P_set_spp  = P_avail_spp;"
        "P_set_bess = min(max(P_ref - P_avail_wpp - P_avail_spp, -P_chg_max_bess), P_dis_max_bess);"
        "Q_set_wpp  = 0;"
        "Q_set_spp  = 0;"
        "Q_set_bess = 0;"];
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
    fb = mdl + "/logic";
    add_block("simulink/User-Defined Functions/MATLAB Function", fb, "Position", [300 30 520 30+h]);
    rt = sfroot;
    ch = rt.find("-isa", "Stateflow.EMChart", "Path", char(fb));
    sig = "function [" + strjoin(outNames, ", ") + "] = logic(" + strjoin(inNames, ", ") + ", par)";
    ch.Script = char(strjoin([sig; "%#codegen"; s.code; "end"], newline));
    d = ch.find("-isa", "Stateflow.Data", "Name", "par");
    d.Scope = 'Parameter';

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
        "Description", type + " asset: " + ctrl + " (asset controller) + placeholder generator model. " + ...
                       "Per unit on the asset rating. Argument ap = p." + t + ".ap(k).");

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

    % inports
    ins = ["P_set", "Q_set", "V_term"];
    if type == "WT", ins(end+1) = "P_wind_avail"; end
    if type == "PV", ins(end+1) = "P_pv_avail"; end
    for k = 1:numel(ins)
        add_block("simulink/Sources/In1", mdl + "/" + ins(k), "Port", num2str(k), ...
            "OutDataTypeStr", "double", "PortDimensions", "1", "Position", [40, 50*k, 70, 50*k+14]);
    end

    % asset controller instance
    cb = mdl + "/ctrl";
    add_block("simulink/Ports & Subsystems/Model", cb, "ModelName", ctrl, "Position", [220 40 380 340]);
    Simulink.ModelReference.refresh(cb);
    setArg(cb, "par", "ap.ctrl");

    % generator placeholder: P = ip*V, Q = iq*V; measurements delayed one step
    add_block("simulink/Math Operations/Product", mdl + "/P gen", "Position", [480 60 510 90]);
    add_block("simulink/Math Operations/Product", mdl + "/Q gen", "Position", [480 130 510 160]);
    add_block("simulink/Discrete/Memory", mdl + "/P meas", "Position", [480 260 510 290]);
    add_block("simulink/Discrete/Memory", mdl + "/Q meas", "Position", [480 320 510 350]);
    L = @(a, b) add_line(mdl, a, b, "autorouting", "smart");
    L("P_set/1", "ctrl/1"); L("Q_set/1", "ctrl/2"); L("V_term/1", "ctrl/3");
    L("P meas/1", "ctrl/4"); L("Q meas/1", "ctrl/5");
    L("ctrl/1", "P gen/1"); L("V_term/1", "P gen/2");
    L("ctrl/2", "Q gen/1"); L("V_term/1", "Q gen/2");
    L("P gen/1", "P meas/1"); L("Q gen/1", "Q meas/1");

    outs = ["P", "Q", "P_avail"];
    for k = 1:numel(outs)
        add_block("simulink/Sinks/Out1", mdl + "/" + outs(k), "Port", num2str(k), ...
            "Position", [640, 60*k, 670, 60*k+14]);
    end
    L("P gen/1", "P/1"); L("Q gen/1", "Q/1"); L("ctrl/3", "P_avail/1");

    switch type
        case "WT", L("P_wind_avail/1", "ctrl/6");
        case "PV", L("P_pv_avail/1", "ctrl/6");
        case "BESS"
            % battery energy: dSOC/dt = -kSOC * P  (P in pu, + = discharge)
            add_block("simulink/Math Operations/Gain", mdl + "/kSOC", "Gain", "-ap.kSOC", "Position", [560 380 600 410]);
            add_block("simulink/Continuous/Integrator", mdl + "/SOC int", "InitialCondition", "ap.SOC0", ...
                "Position", [640 380 670 410]);
            add_block("simulink/Sinks/Out1", mdl + "/SOC", "Port", "4", "Position", [760 388 790 402]);
            L("P gen/1", "kSOC/1"); L("kSOC/1", "SOC int/1"); L("SOC int/1", "SOC/1"); L("SOC int/1", "ctrl/6");
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
    T.const("V_poc", "1", [30 350]);          T.line("V_poc/1", "HPPC/6");

    % ---------------- plants ----------------
    plants = [ ...
        struct("tag","wpp",  "ctrl","WPPC",  "c","wppc",  "type","WT",   "t","wt",   "y",  40, "hppIn",[7 12 13],   "hppOut",[1 2]), ...
        struct("tag","spp",  "ctrl","SPPC",  "c","sppc",  "type","PV",   "t","pv",   "y", 900, "hppIn",[8 14 15],   "hppOut",[3 4]), ...
        struct("tag","bess", "ctrl","BESSC", "c","bessc", "type","BESS", "t","bess", "y",1760, "hppIn",[0 16 17],  "hppOut",[5 6])];
    sumP = strings(1, 3); sumQ = strings(1, 3);
    for i = 1:numel(plants)
        [sumP(i), sumQ(i)] = buildPlant(T, p, plants(i), hb);
    end

    % ---------------- POC (stage 1: lossless sum, no network yet) ----------------
    T.block("simulink/Math Operations/Sum", "P_poc sum", [1900 300 1930 380], "Inputs", "+++");
    T.block("simulink/Math Operations/Sum", "Q_poc sum", [1900 440 1930 520], "Inputs", "+++");
    for i = 1:3
        T.line(sumP(i), sprintf("P_poc sum/%d", i));
        T.line(sumQ(i), sprintf("Q_poc sum/%d", i));
    end
    T.chain("P_poc sum/1", ["gain:1/" + hb, "memory", "delay:p.comm.meas_hppc"], "HPPC/4", "P_poc meas");
    T.chain("Q_poc sum/1", ["gain:1/" + hb, "memory", "delay:p.comm.meas_hppc"], "HPPC/5", "Q_poc meas");

    % ---------------- outports (logged as out.yout) ----------------
    T.outport("P_poc", "P_poc sum/1", 1);
    T.outport("SOC", "SOC plant/1", 2);
    T.block("simulink/Math Operations/Gain", "P_bess_set to W", [900 600 960 630], "Gain", hb);
    T.line("HPPC/5", "P_bess_set to W/1");
    T.outport("P_bess_set", "P_bess_set to W/1", 3);
    T.outport("Q_poc", "Q_poc sum/1", 4);
    for i = 1:3
        T.outport("P_" + plants(i).tag, sumP(i), 4 + i);
    end

    % record what this build was made for; runCase/runSweep check it
    ws = get_param(m, "ModelWorkspace");
    assignin(ws, "buildInfo", struct("N_wt", p.wt.N, "N_pv", p.pv.N, "N_bess", p.bess.N, ...
        "built", string(datetime("now"))));
    save_system(m, fullfile(folder, "HPP.slx"));
    fprintf("Built HPP.slx with N = %d wind, %d PV, %d BESS asset instances.\n", p.wt.N, p.pv.N, p.bess.N);
end

function [sumPout, sumQout] = buildPlant(T, p, pl, hb)
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
    nOutA = 3 + isB;
    muxNames = ["P", "Q", "P_avail", "SOC"];
    for j = 1:nOutA
        T.block("simulink/Signal Routing/Mux", pl.tag + " " + muxNames(j) + "_k", ...
            [1720 y+60*(j-1) 1725 y+60*(j-1)+50], "Inputs", num2str(N));
    end
    for k = 1:N
        a = sprintf("%s %d", pl.type, k);
        ya = y + 300 + 130*(k-1);
        T.model(a, pl.type + "_asset", [1480 ya 1620 ya+100]);
        setArg(T.m + "/" + a, "ap", sprintf("p.%s.ap(%d)", pl.t, k));
        T.line(sprintf("%s P_set_k/%d", C, k), a + "/1");
        T.line(sprintf("%s Q_set_k/%d", C, k), a + "/2");
        T.line("V_term/1", a + "/3");
        switch pl.type
            case "WT"
                src = sprintf("wind %d", k);
                T.block("simulink/Sources/Step", src, [1300 ya+60 1330 ya+90], "Time", "p.wt.tStep", ...
                    "Before", "p.wt.Pavail0", "After", "p.wt.PavailStep", "SampleTime", "0");
                T.line(src + "/1", a + "/4");
            case "PV"
                src = sprintf("irradiance %d", k);
                T.const(src, "p.pv.Pavail", [1300 ya+60]);
                T.line(src + "/1", a + "/4");
        end
        for j = 1:nOutA
            T.line(sprintf("%s/%d", a, j), sprintf("%s %s_k/%d", pl.tag, muxNames(j), k));
        end
    end

    % plant P and Q in W (stage 1: no network losses)
    sumPout = T.sumW(pl.tag + " P_k/1", ab, "P_" + pl.tag + " W");
    sumQout = T.sumW(pl.tag + " Q_k/1", ab, "Q_" + pl.tag + " W");

    % measurements -> plant controller
    T.chain(sumPout, ["gain:1/" + pb, "memory", "delay:p.comm.meas_plant"], C + "/3", "P_meas " + pl.tag);
    T.chain(sumQout, ["gain:1/" + pb, "memory", "delay:p.comm.meas_plant"], C + "/4", "Q_meas " + pl.tag);
    T.line("V_term/1", C + "/5");
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
    T.sumW  = @(src, base, label) sumW(m, src, base, label, @uniq);
    T.outport = @(name, src, port) outport(m, name, src, port);

    if ~bdIsLoaded(m), error("model not loaded"); end
    add_block("simulink/Sources/Constant", m + "/V_term", "Value", "1", "Position", [1300 0 1360 30]);
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

function out = sumW(m, src, base, label, uniq)
    % sum the elements of a vector (pu on base) and convert to W
    s = uniq(label + " sum");
    pp = get_param(m + "/" + extractBefore(src, "/"), "Position");
    add_block("simulink/Math Operations/Sum", m + "/" + s, "Inputs", "+", "Position", [pp(3)+30 pp(2) pp(3)+60 pp(2)+30]);
    g = uniq(label);
    add_block("simulink/Math Operations/Gain", m + "/" + g, "Gain", base, "Position", [pp(3)+90 pp(2) pp(3)+140 pp(2)+30]);
    add_line(m, src, s + "/1", "autorouting", "smart");
    h = add_line(m, s + "/1", g + "/1", "autorouting", "smart");
    set_param(h, "Name", label);
    out = g + "/1";
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
