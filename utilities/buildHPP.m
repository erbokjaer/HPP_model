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
                buildController(specs(k), dirs.ctrl);
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
        add_block("simulink/Sources/Constant", sys + "/" + names(j), "Value", vals(j), "Position", [0 0 90 26]);
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
    addMatlabFunction(sys + "/solve", "solve", ["I_re", "I_im"], outs, body, "net", [0 0 180 320]);
    ln(sys, "I_re all/1", "solve/1", "I_re");
    ln(sys, "I_im all/1", "solve/2", "I_im");

    % terminal voltages one step old (generator "PLL"; avoids an algebraic loop)
    add_block("simulink/Discrete/Memory", sys + "/V_re prev", "InitialCondition", "ones(net.nA,1)");
    add_block("simulink/Discrete/Memory", sys + "/V_im prev", "InitialCondition", "zeros(net.nA,1)");
    ln(sys, "solve/1", "V_re prev/1", "");
    ln(sys, "solve/2", "V_im prev/1", "");
    % buses out; powers from system pu to W / var
    busCreator(sys, "network bus", 5);
    ln(sys, "V_re prev/1", "network bus/1", "V_re");
    ln(sys, "V_im prev/1", "network bus/2", "V_im");
    chain(sys, "solve/3", "gain:net.Sb", "network bus/3", "P_plant to W", "P_plant");
    chain(sys, "solve/4", "gain:net.Sb", "network bus/4", "Q_plant to var", "Q_plant");
    ln(sys, "solve/5", "network bus/5", "V_bus");
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
    ln(mdl, "ctrl/1", "generator/1", "ip_cmd");
    ln(mdl, "ctrl/2", "generator/2", "iq_cmd");
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
    nCin = 6 + isB;  nCout = 5 + 3*isB;
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
    netEl = ["V_re", "V_im", "P_plant", "Q_plant", "V_bus"];
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
    for j = 1:5
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
