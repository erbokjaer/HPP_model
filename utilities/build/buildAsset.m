function buildAsset(type, folder)
    mdl = string(type + "_asset");
    ctrl = type + "_ctrl";
    t = lower(type);
    closeIfLoaded(mdl);
    new_system(mdl);
    set_param(mdl, "SolverType", "Fixed-step", "Solver", "ode4", "FixedStep", "Tsim", "StopTime", "inf", ...
        "Description", type + " asset: " + ctrl + " (asset controller) + generator / converter model " + ...
                       "(WT: IEC 61400-27-1 Type 4 generator system; PV, BESS: WECC REGC_A) feeding a " + ...
                       "current source along the angle of the controller's PLL. Per unit on the asset rating " + ...
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
               "P_wind_avail", "P_pv_avail", "I_re", "I_im", "th_pll", "c", "s", "f_pll"];
    ins = ["P_set", "Q_set", "V_re", "V_im"];
    if src ~= "", ins(end+1) = src; end
    [hIn, x] = ctrlInputs(mdl, ins, repmat("1", 1, numel(ins)), cm, "-1");

    % terminal measurements: |V| = hypot(V_re, V_im), P and Q one step old
    nb(mdl, "math", "|V|", "Operator", "hypot");
    nb(mdl, "mem", "P meas");  nb(mdl, "mem", "Q meas");
    rowsM = {
        [stg("@V_re", 0), stg("|V|", 1, {2, "@V_im"}), stg(">V_term")]
        [stg("@P", 0), stg("P meas"), stg(">P_meas")]
        [stg("@Q", 0), stg("Q meas"), stg(">Q_meas")]};
    [y, h1, r1] = ctrlSection(mdl, "Terminal measurements", rowsM, x, 20, cm, areaColor("interface"));

    % asset controller (future DLL), ports in 1 P_set 2 Q_set 3 V_term 4 P_meas
    % 5 Q_meas 6 source 7 V_re 8 V_im; out 1 ip_cmd 2 iq_cmd 3 P_avail 4 th_pll
    % (its PLL: the angle the converter injects along, rad) 5 f_pll
    cb = mdl + "/ctrl";
    add_block("simulink/Ports & Subsystems/Model", cb, "ModelName", ctrl, "Position", [0 0 180 210]);
    Simulink.ModelReference.refresh(cb);
    setArg(cb, "par", "ap.ctrl");
    set_param(cb, "BackgroundColor", areaColor("controller"));
    if src ~= "", s6 = "@" + src; else, s6 = "@SOC"; end
    % f_pll: a label for asset-level frequency functions later
    rowsC = {[stg("@P_set", 0), stg("ctrl", 1, {2, "@Q_set"; 3, "@V_term"; 4, "@P_meas"; 5, "@Q_meas"; 6, s6; ...
              7, "@V_re"; 8, "@V_im"}, {2, "iq_cmd"; 3, "P_avail"; 4, "th_pll"; 5, "f_pll"}), stg(">ip_cmd")]};
    [y, h2, r2] = ctrlSection(mdl, "Asset controller (" + ctrl + ")", rowsC, x, y, cm, areaColor("controller"));

    % three blocks (virtual subsystems) between the controller and the
    % outputs:
    %  - generator system (WT, IEC 61400-27-1) / converter (PV, BESS, WECC
    %    REGC_A): the current dynamics, ip_cmd, iq_cmd -> ip, iq;
    %  - current injection, the converter model chosen per run by the
    %    tunable ap.conv.type (converterParameters):
    %    Current source (1, RMS, default): a current source along the PLL
    %    angle th_pll, I = (ip - j iq) e^(j th), with c = cos th, s = sin th
    %      I_re = ip c + iq s,  I_im = ip s - iq c;
    %    Averaged (2): averaged converter with inner dq current loop, LCL
    %    filter and DC link tracking the same current; the network sees its
    %    grid-side current (needs the terminal voltage V_re, V_im);
    %  - PQ measurement: P + jQ = V conj(I) (+ = export / capacitive)
    %      P = V_re I_re + V_im I_im,  Q = V_im I_re - V_re I_im
    %    (with an ideal PLL, th = angle of V: P = |V| ip, Q = |V| iq).
    if type == "WT", gName = "Generator system"; else, gName = "Converter"; end
    gs = mdl + "/" + gName;
    add_block("built-in/Subsystem", gs, "Position", [0 0 120 90]);
    ij = mdl + "/Current injection";
    cs = currentInjectionBlock(ij);
    pq = mdl + "/PQ measurement";
    add_block("built-in/Subsystem", pq, "Position", [0 0 120 120]);
    nb(cs, "trig", "cos", "Operator", "cos");  nb(cs, "trig", "sin", "Operator", "sin");
    for nm = ["ip c", "iq s", "ip s", "iq c"], nb(cs, "prod", nm); end
    nb(cs, "sum", "I_re sum", "Inputs", "++");  nb(cs, "sum", "I_im sum", "Inputs", "+-");
    for nm = ["V_re I_re", "V_im I_im", "V_im I_re", "V_re I_im"], nb(pq, "prod", nm); end
    nb(pq, "sum", "P sum", "Inputs", "++");     nb(pq, "sum", "Q sum", "Inputs", "+-");
    if type == "WT"
        % IEC 61400-27-1 Type 4 generator system: current rate limits, then
        % a first-order lag Tg (active current: only its rise is limited)
        nb(gs, "rate", "dipmax", "RisingSlewLimit", "ap.gen.dipmax", "FallingSlewLimit", "-1e6");
        contLag(gs, "Tg ip", "ap.gen.Tg");
        nb(gs, "rate", "diqmax", "RisingSlewLimit", "ap.gen.diqmax", "FallingSlewLimit", "-ap.gen.diqmax");
        contLag(gs, "Tg iq", "ap.gen.Tg");
        rowsG = [{
            [stg("@ip_cmd", 0), stg("dipmax"), stg("Tg ip"), stg(">ip")]
            [stg("@iq_cmd", 0), stg("diqmax"), stg("Tg iq"), stg(">iq")]}];
        gTitle = "Generator system (IEC 61400-27-1 Type 4)";
    else
        % WECC REGC_A: Ip recovery ramp Rrpwr, Iq ramps Iqrmax/Iqrmin, lag Tg;
        % low voltage active current management (LVACM) and low voltage
        % power logic (LVPL, static, on the unfiltered voltage)
        nb(gs, "rate", "Rrpwr", "RisingSlewLimit", "ap.gen.Rrpwr", "FallingSlewLimit", "-1e6");
        contLag(gs, "Tg ip", "ap.gen.Tg");
        nb(gs, "rate", "Iqrmax", "RisingSlewLimit", "ap.gen.Iqrmax", "FallingSlewLimit", "ap.gen.Iqrmin");
        contLag(gs, "Tg iq", "ap.gen.Tg");
        % LVPL: ipl = 0 for |V| <= Zerox, Lvpl1 (|V| - Zerox) / (Brkpt - Zerox)
        % up to Brkpt, no limit (1e6) above or with Lvplsw = 0;
        % ip_l = sat(ip, +-ipl). LVACM: ip = k ip_l, k = sat((|V| - Lvpnt0) /
        % (Lvpnt1 - Lvpnt0), 0, 1). |V| = V_term (unfiltered).
        kc(gs, "Zerox", "ap.gen.Zerox");  kc(gs, "Brkpt - Zerox", "ap.gen.Brkpt - ap.gen.Zerox");
        kc(gs, "Brkpt", "ap.gen.Brkpt");  kc(gs, "zero ipl", "0");  kc(gs, "no limit", "1e6");
        kc(gs, "no limit 2", "1e6");  kc(gs, "Lvplsw", "ap.gen.Lvplsw");
        kc(gs, "Lvpnt0", "ap.gen.Lvpnt0");  kc(gs, "Lvpnt1 - Lvpnt0", "ap.gen.Lvpnt1 - ap.gen.Lvpnt0");
        nb(gs, "sum", "V - Zerox", "Inputs", "+-");
        nb(gs, "gain", "Lvpl1", "Gain", "ap.gen.Lvpl1");
        nb(gs, "div", "÷ (Brkpt - Zerox)");
        nb(gs, "minmax", "max 0", "Function", "max", "Inputs", "2");
        nb(gs, "rel", "V < Brkpt", "Operator", "<");
        nb(gs, "switch", "below Brkpt?", "Criteria", "u2 ~= 0");
        nb(gs, "switch", "LVPL on?", "Criteria", "u2 ~= 0");
        nb(gs, "gain", "-ipl", "Gain", "-1");
        nb(gs, "satdyn", "ip limit");
        nb(gs, "sum", "V - Lvpnt0", "Inputs", "+-");
        nb(gs, "div", "÷ (Lvpnt1 - Lvpnt0)");
        nb(gs, "sat", "k 0..1", "UpperLimit", "1", "LowerLimit", "0");
        nb(gs, "prod", "k ip");
        rowsLV = {
            [stg("@V_term", 0), stg("V - Zerox", 1, {2, "Zerox"}), stg("Lvpl1"), stg("÷ (Brkpt - Zerox)", 1, {2, "Brkpt - Zerox"}), ...
             stg("max 0", 1, {2, "zero ipl"}), stg(">ipl_lin")]
            [stg("@V_term", 0), stg("V < Brkpt", 1, {2, "Brkpt"}), stg(">below_brk")]
            [stg("@ipl_lin", 0), stg("below Brkpt?", 1, {2, "@below_brk"; 3, "no limit"}), stg("LVPL on?", 1, {2, "Lvplsw"; 3, "no limit 2"}), stg(">ipl")]
            [stg("@ipl", 0), stg("-ipl"), stg(">ipl_neg")]
            [stg("@ip_raw", 0), stg("ip limit", 2, {1, "@ipl"; 3, "@ipl_neg"}), stg(">ip_l")]
            [stg("@V_term", 0), stg("V - Lvpnt0", 1, {2, "Lvpnt0"}), stg("÷ (Lvpnt1 - Lvpnt0)", 1, {2, "Lvpnt1 - Lvpnt0"}), stg("k 0..1"), stg(">k_lvacm")]
            [stg("@ip_l", 0), stg("k ip", 1, {2, "@k_lvacm"}), stg(">ip")]};
        rowsG = [{
            [stg("@ip_cmd", 0), stg("Rrpwr"), stg("Tg ip"), stg(">ip_raw")]
            [stg("@iq_cmd", 0), stg("Iqrmax"), stg("Tg iq"), stg(">iq")]}; rowsLV];
        gTitle = "Converter (WECC REGC_A)";
    end
    % inside the blocks: inputs -> the rows above -> outputs
    gin = ["ip_cmd", "iq_cmd"];
    if type ~= "WT", gin(end+1) = "V_term"; end      % LVPL, LVACM
    fillBlock(gs, gin, ["ip", "iq"], rowsG, gTitle, cm);
    layoutConverter(gs, type == "WT", gTitle);
    % (pairBlock: the hand layout of pairs of products into round sums)
    pairBlock(cs, ["ip", "iq", "th_pll"], ["I_re", "I_im"], {"th_pll", "cos", "c"; "th_pll", "sin", "s"}, ...
        {"ip c", "ip", "c", "iq s", "iq", "s", "I_re sum", "+", "I_re"
         "ip s", "ip", "s", "iq c", "iq", "c", "I_im sum", "-", "I_im"}, ...
        "Current injection along the PLL angle: I = (ip - j iq) e^(j th)", cm);
    averagedConverter(ij + "/Averaged", cm);
    wireCurrentInjection(ij);
    pairBlock(pq, ["I_re", "I_im", "V_re", "V_im"], ["P", "Q"], {}, ...
        {"V_re I_re", "V_re", "I_re", "V_im I_im", "V_im", "I_im", "P sum", "+", "P"
         "V_im I_re", "V_im", "I_re", "V_re I_im", "V_re", "I_im", "Q sum", "-", "Q"}, ...
        "PQ measurement: P + jQ = V conj(I)", cm);
    % in the asset: one row, generator -> injection -> PQ measurement
    sG = {2, "@iq_cmd"};
    if type ~= "WT", sG(end+1, :) = {3, "@V_term"}; end
    rowsGB = {[stg("@ip_cmd", 0), stg(gName, 1, sG, {2, "iq"}), ...
               stg("Current injection", 1, {2, "@iq"; 3, "@th_pll"; 4, "@V_re"; 5, "@V_im"}, {1, "I_re"; 2, "I_im"}), ...
               stg("PQ measurement", 1, {2, "@I_im"; 3, "@V_re"; 4, "@V_im"}, {2, "Q"}), stg(">P")]};
    [y, h3, r3] = ctrlSection(mdl, gTitle, rowsGB, x, y, cm, areaColor("asset"));
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
    % layout: the template models/layout/asset.json (the WT_asset hand
    % layout); PV and BESS by role; the converter's V_term as a label
    f = fullfile(currentProject().RootFolder, "models", "layout", "asset.json");
    if isfile(f)
        if type == "WT"
            r = applyLayout(mdl, f);
        else
            m = ["Generator system", "Converter"; "P_wind_avail", src
                 "Generator system (IEC 61400-27-1 Type 4)", gTitle];
            if src == "", m = m([1 3], :); end
            tg = dictionary("|V|", "V_term");  pl = cell(0, 3);
            if isB
                % battery energy at the bottom of the converter area; P in
                % and SOC out as labels (SOC feeds the controller and the
                % SOC outport, a line would cross the converter)
                tg = dictionary(["|V|", "SOC int", "PQ measurement:1"], ["V_term", "SOC", "P"]);
                pl = {"kSOC", 1010, 400;  "SOC int", 1100, 400;  "SOC", 1685, 350};
            end
            r = applyLayout(mdl, f, Map=dictionary(m(:, 1), m(:, 2)), Tags=tg, Place=pl);
        end
        if ~isempty(r.missing),  fprintf("  %s template: not here: %s\n", mdl, strjoin(r.missing, ", ")); end
        if ~isempty(r.unplaced), fprintf("  %s template: placed by its neighbours: %s\n", mdl, strjoin(r.unplaced, ", ")); end
    end
    save_system(mdl, fullfile(folder, mdl + ".slx"));
    close_system(mdl);
    registerInProject(fullfile(folder, mdl + ".slx"));
end

function cs = currentInjectionBlock(ij)
    % the Current injection: both converter models side by side and a
    % Switch per output on the tunable ap.conv.type (1 Current source,
    % 2 Averaged). Ports in 1 ip 2 iq 3 th_pll 4 V_re 5 V_im, out 1 I_re
    % 2 I_im. Not a Variant Subsystem: a referenced model with an update
    % diagram (or startup) variant built for Rapid Accelerator only with
    % the choice that was active when the model was saved ("computed
    % interface information must match saved interface information"), so
    % switching per run would need the asset models resaved. With the
    % switch both are compiled, the choice needs no rebuild (also in Fast
    % Restart), and the averaged converter returns at once when not
    % selected. Returns the Current source block (filled by pairBlock).
    add_block("built-in/Subsystem", ij, "Position", [0 0 120 130]);
    ins = ["ip", "iq", "th_pll", "V_re", "V_im"];
    for k = 1:numel(ins)
        add_block("simulink/Sources/In1", ij + "/" + ins(k), "Port", num2str(k), "Position", [40, 40 * k + 20, 70, 40 * k + 34]);
    end
    cs = ij + "/Current source";
    add_block("built-in/Subsystem", cs, "Position", [200 40 360 130]);
    add_block("built-in/Subsystem", ij + "/Averaged", "Position", [200 180 360 300]);
    add_block("simulink/Sources/Constant", ij + "/converter model", "Value", "ap.conv.type", ...
        "Position", [420 150 490 170]);
    outs = ["I_re", "I_im"];
    for k = 1:2
        sw = outs(k) + " model?";
        add_block("simulink/Signal Routing/Switch", ij + "/" + sw, "Criteria", "u2 > Threshold", "Threshold", "1.5", ...
            "Position", [560, 60 + 120 * (k - 1), 590, 150 + 120 * (k - 1)]);
        add_block("simulink/Sinks/Out1", ij + "/" + outs(k), "Port", num2str(k), ...
            "Position", [650, 98 + 120 * (k - 1), 680, 112 + 120 * (k - 1)]);
        add_line(ij, sw + "/1", outs(k) + "/1");
        add_line(ij, "converter model/1", sw + "/2", "autorouting", "on");
    end
    set_param(ij, "BackgroundColor", areaColor("asset"), "Description", ...
        "Converter model, chosen per run by p.conv.type: Current source (1, RMS: I = (ip - j iq) e^(j th_pll)) " + ...
        "or Averaged (2: inner dq current loop, LCL filter, DC link; params/converterParameters.m).");
end

function wireCurrentInjection(ij)
    % after both blocks are filled: inputs into them, their outputs into
    % the switches (u1 Averaged, u3 Current source)
    ins = ["ip", "iq", "th_pll", "V_re", "V_im"];
    for k = 1:5
        add_line(ij, ins(k) + "/1", "Averaged/" + k, "autorouting", "on");
        if k <= 3, add_line(ij, ins(k) + "/1", "Current source/" + k, "autorouting", "on"); end
    end
    outs = ["I_re", "I_im"];
    for k = 1:2
        add_line(ij, "Averaged/" + k, outs(k) + " model?/1", "autorouting", "on");
        add_line(ij, "Current source/" + k, outs(k) + " model?/3", "autorouting", "on");
    end
end

function averagedConverter(sub, cm)
    % Averaged block of the Current injection: one MATLAB Function in an
    % atomic subsystem at the model step Tsim, with ap.conv.n sub-steps
    % (avgConverterCode); the DC voltage Vdc ends in a terminator (not an
    % asset output)
    [hI, x] = ctrlInputs(sub, ["ip", "iq", "th_pll", "V_re", "V_im"], repmat("1", 1, 5), cm, "-1");
    fb = sub + "/averaged converter";
    addMatlabFunction(fb, "avg_converter", ["ip", "iq", "th", "V_re", "V_im"], ["I_re", "I_im", "Vdc"], ...
        avgConverterCode(), "ap", [x + 260, 60, x + 420, 260]);
    tags = ["ip", "iq", "th_pll", "V_re", "V_im"];
    for k = 1:5
        f = addFrom(sub, tags(k), tagColor(cm, tags(k)));
        placeLeft(sub, f, fb, k, 60);
        add_line(sub, f + "/1", "averaged converter/" + k);
    end
    add_block("simulink/Sinks/Terminator", sub + "/Vdc term");
    q = get_param(fb, "PortHandles");  po = get_param(q.Outport(3), "Position");
    set_param(sub + "/Vdc term", "Position", [po(1) + 60, po(2) - 10, po(1) + 80, po(2) + 10]);
    hL = add_line(sub, "averaged converter/3", "Vdc term/1");
    set_param(hL, "Name", "Vdc");
    outs = ["I_re", "I_im"];
    for k = 1:2
        g = addGoto(sub, outs(k), tagColor(cm, outs(k)));
        p = get_param(sub + "/" + g, "Position");  w = p(3) - p(1);  h = p(4) - p(2);
        po = get_param(q.Outport(k), "Position");
        set_param(sub + "/" + g, "Position", [po(1) + 60, po(2) - h / 2, po(1) + 60 + w, po(2) + h / 2]);
        add_line(sub, "averaged converter/" + k, g + "/1");
    end
    blks = [string(get_param(find_system(sub, "SearchDepth", 1, "BlockType", "From"), "Name"))', ...
        "averaged converter", "Vdc term", "I_re goto", "I_im goto"];
    h = areaAround(sub, "Averaged converter (inner current loop, LCL filter, DC link)", blks, areaColor("asset"));
    r = max(cell2mat(arrayfun(@(b) get_param(sub + "/" + b, "Position"), blks', "UniformOutput", false)) * [0; 0; 1; 0]);
    hO = ctrlOutputs(sub, outs, "@" + outs, r + 110, cm);
    alignColumns(hI, h, hO);
    % atomic at the model step: the MATLAB Function keeps discrete states
    % (persistent) and must not inherit the continuous rate of ip, iq
    set_param(sub, "TreatAsAtomicUnit", "on", "SystemSampleTime", "Tsim");
    set_param(sub, "BackgroundColor", areaColor("asset"), "Description", ...
        "Averaged converter: inner dq current loop in the PLL frame, LCL filter, DC link, " + ...
        "angle-preserving current limit (params/converterParameters.m)");
end

function layoutConverter(gs, isWT, title)
    % inside of the Generator system (WT) / Converter (PV, BESS) block, in
    % the style of the BESS_ctrl hand layout (docs/layout_rules.md): rows
    % with their ports level, lines; labels only where a line would cross
    b = @(n) gs + "/" + n;
    if isWT
        % IEC Type 4: two rows, ip and iq
        putAt(gs, "dipmax", 200, "in", 1, 60);  putAt(gs, "Tg ip", 290, "in", 1, 60);
        putAt(gs, "diqmax", 200, "in", 1, 140);  putAt(gs, "Tg iq", 290, "in", 1, 140);
        putAt(gs, "ip_cmd", 40, "out", 1, 60);  putAt(gs, "iq_cmd", 40, "out", 1, 140);
        putAt(gs, "ip", 440, "in", 1, portY(b("Tg ip"), "out", 1));
        putAt(gs, "iq", 440, "in", 1, portY(b("Tg iq"), "out", 1));
        main = ["dipmax", "Tg ip", "diqmax", "Tg iq"];
        tags = dictionary(string.empty, string.empty);  keep = strings(1, 0);
    else
        % REGC_A: LVPL on top, its switches stepped down so that ipl meets
        % the ip limit's upper port; the ip row into the ip limit and k ip;
        % -ipl into the lower port; LVACM below, up into k ip; iq at the
        % bottom
        for n = ["below Brkpt?", "LVPL on?"], set_param(b(n), "Position", [0 0 40 120]); end
        set_param(b("ip limit"), "Position", [0 0 50 270]);   % its u port well below LVPL on?
        yA = 80;
        putAt(gs, "V - Zerox", 360, "in", 1, yA);  placeLeft(gs, "Zerox", b("V - Zerox"), 2, 15);
        putAt(gs, "Lvpl1", 450, "in", 1, portY(b("V - Zerox"), "out", 1));
        putAt(gs, "÷ (Brkpt - Zerox)", 790, "in", 1, portY(b("Lvpl1"), "out", 1));
        placeLeft(gs, "Brkpt - Zerox", b("÷ (Brkpt - Zerox)"), 2, 15);
        putAt(gs, "max 0", 920, "in", 1, portY(b("÷ (Brkpt - Zerox)"), "out", 1));
        placeLeft(gs, "zero ipl", b("max 0"), 2, 15);
        putAt(gs, "below Brkpt?", 1020, "in", 1, portY(b("max 0"), "out", 1));
        placeLeft(gs, "no limit", b("below Brkpt?"), 3, 15);
        putAt(gs, "LVPL on?", 1260, "in", 1, portY(b("below Brkpt?"), "out", 1));
        placeLeft(gs, "Lvplsw", b("LVPL on?"), 2, 15);  placeLeft(gs, "no limit 2", b("LVPL on?"), 3, 15);
        putAt(gs, "ip limit", 1420, "in", 1, portY(b("LVPL on?"), "out", 1));
        yIp = portY(b("ip limit"), "in", 2);
        putAt(gs, "V < Brkpt", 360, "in", 1, yA + 70);  placeLeft(gs, "Brkpt", b("V < Brkpt"), 2, 15);
        putAt(gs, "Rrpwr", 360, "in", 1, yIp);  putAt(gs, "Tg ip", 460, "in", 1, yIp);
        putAt(gs, "-ipl", 1280, "out", 1, portY(b("ip limit"), "in", 3));
        putAt(gs, "k ip", 1530, "in", 1, portY(b("ip limit"), "out", 1));
        yL = portY(b("ip limit"), "in", 3) + 70;
        putAt(gs, "V - Lvpnt0", 360, "in", 1, yL);  placeLeft(gs, "Lvpnt0", b("V - Lvpnt0"), 2, 15);
        putAt(gs, "÷ (Lvpnt1 - Lvpnt0)", 740, "in", 1, portY(b("V - Lvpnt0"), "out", 1));
        placeLeft(gs, "Lvpnt1 - Lvpnt0", b("÷ (Lvpnt1 - Lvpnt0)"), 2, 15);
        putAt(gs, "k 0..1", 830, "in", 1, portY(b("÷ (Lvpnt1 - Lvpnt0)"), "out", 1));
        yIq = yL + 70;
        putAt(gs, "Iqrmax", 360, "in", 1, yIq);  putAt(gs, "Tg iq", 460, "in", 1, yIq);
        putAt(gs, "ip_cmd", 40, "out", 1, yIp);  putAt(gs, "iq_cmd", 40, "out", 1, yIq);
        putAt(gs, "V_term", 40, "out", 1, yA);
        putAt(gs, "ip", 1650, "in", 1, portY(b("k ip"), "out", 1));
        putAt(gs, "iq", 1650, "in", 1, portY(b("Tg iq"), "out", 1));
        main = ["V - Zerox", "Zerox", "Lvpl1", "÷ (Brkpt - Zerox)", "Brkpt - Zerox", "max 0", "zero ipl", ...
                "below Brkpt?", "no limit", "LVPL on?", "Lvplsw", "no limit 2", "ip limit", "V < Brkpt", "Brkpt", ...
                "Rrpwr", "Tg ip", "-ipl", "k ip", "V - Lvpnt0", "Lvpnt0", "÷ (Lvpnt1 - Lvpnt0)", ...
                "Lvpnt1 - Lvpnt0", "k 0..1", "Iqrmax", "Tg iq"];
        % V_term feeds three rows; below_brk would cross the LVPL row; -ipl
        % below the ip row (a line from ipl would cross it)
        tags = dictionary(["V_term", "V < Brkpt", "LVPL on?"], ["V_term", "below_brk", "ipl"]);
        keep = ["V_term->V - Zerox", "LVPL on?->ip limit"];
    end
    % areas: inputs | the block's function | outputs
    ins = string(get_param(find_system(gs, "SearchDepth", 1, "BlockType", "Inport"), "Name"))';
    outs = string(get_param(find_system(gs, "SearchDepth", 1, "BlockType", "Outport"), "Name"))';
    box = @(n) cell2mat(arrayfun(@(x) get_param(b(x), "Position"), n(:), "UniformOutput", false));
    Pm = box(main);  Pi = box(ins);  Po = box(outs);
    top = min([Pm(:, 2); Pi(:, 2); Po(:, 2)]) - 40;  bot = max([Pm(:, 4); Pi(:, 4); Po(:, 4)]) + 30;
    A = struct("title", {"Inputs", title, "Outputs"}, ...
        "pos", {[min(Pi(:, 1)) - 20, top, max(Pi(:, 3)) + 30, bot], ...
                [max(min(Pm(:, 1)) - 140, max(Pi(:, 3)) + 50), top, max(Pm(:, 3)) + 20, bot], ...
                [max(min(Po(:, 1)) - 20, max(Pm(:, 3)) + 40), top, max(Po(:, 3)) + 20, bot]}, ...
        "color", {areaColor("interface"), areaColor("asset"), areaColor("interface")});
    applyCodedLayout(gs, A, tags, keep);
    if ~isWT
        % the [ipl] label above its line, so the line into the ip limit's
        % upper port stays straight
        E0 = signalEdges(gs);
        g = find_system(gs, "SearchDepth", 1, "FindAll", "on", "BlockType", "Goto", "GotoTag", "ipl");
        sp = portH2(b("LVPL on?"), "out", 1);  ps = get_param(sp, "Position");
        p = get_param(g, "Position");  w = p(3) - p(1);  h = p(4) - p(2);
        gp = get_param(g, "PortHandles");  delete_line(get_param(gp.Inport, "Line"));
        dp = portH2(b("ip limit"), "in", 1);  delete_line(get_param(dp, "Line"));
        set_param(g, "Position", [ps(1) + 30, ps(2) - 35 - h / 2, ps(1) + 30 + w, ps(2) - 35 + h / 2]);
        add_line(gs, sp, dp, "autorouting", "smart");
        add_line(gs, sp, gp.Inport, "autorouting", "smart");
        if ~isequal(signalEdges(gs), E0), error("layoutConverter:connections", "connections changed in %s", gs); end
    end
end

function p = portH2(blk, kind, k)
    ph = get_param(blk, "PortHandles");
    if kind == "in", p = ph.Inport(k); else, p = ph.Outport(k); end
end

function pairBlock(sub, ins, outs, pre, pairs, title, cm)
    % contents of a block of product pairs (hand layout of PQ measurement):
    % inputs -> Goto; pre rows {from tag, block, Goto tag}; then per pair
    % {prodA, a1, a2, prodB, b1, b2, sum, sign, out}: two product rows, a
    % round sum right of prodA (input from the left: prodA; from below:
    % prodB with sign), Goto out; From -> outputs
    [hI, x0] = ctrlInputs(sub, ins, repmat("1", 1, numel(ins)), cm, "-1");
    y = 60;  blks = strings(0);
    for k = 1:size(pre, 1)                         % e.g. th -> cos -> [c]
        f = addFrom(sub, pre{k, 1}, tagColor(cm, pre{k, 1}));
        b = pre{k, 2};  g = addGoto(sub, pre{k, 3}, tagColor(cm, pre{k, 3}));
        place(sub + "/" + b, x0 + 150, y);  placeLeft(sub, f, sub + "/" + b, 1, 40);
        placeRight(sub, g, sub + "/" + b, 1, 40);
        add_line(sub, f + "/1", b + "/1");  add_line(sub, b + "/1", g + "/1");
        blks = [blks, f, b, g]; %#ok<AGROW>
        y = y + 70;
    end
    for k = 1:size(pairs, 1)
        [pA, a1, a2, pB, b1, b2, sm, sg, out] = pairs{k, :};
        set_param(sub + "/" + sm, "IconShape", "round", "Inputs", "|+" + sg, "Position", [0 0 30 30]);
        for r = 1:2
            if r == 1, p = pA; s1 = a1; s2 = a2; else, p = pB; s1 = b1; s2 = b2; end
            place(sub + "/" + p, x0 + 150, y);
            f1 = addFrom(sub, s1, tagColor(cm, s1));  f2 = addFrom(sub, s2, tagColor(cm, s2));
            placeLeft(sub, f1, sub + "/" + p, 1, 90);  placeLeft(sub, f2, sub + "/" + p, 2, 20);
            add_line(sub, f1 + "/1", p + "/1");  add_line(sub, f2 + "/1", p + "/2");
            blks = [blks, p, f1, f2]; %#ok<AGROW>
            if r == 1
                % the sum level with prodA's output, its Goto right of it
                q = get_param(sub + "/" + p, "PortHandles");  po = get_param(q.Outport, "Position");
                set_param(sub + "/" + sm, "Position", [po(1) + 120, po(2) - 15, po(1) + 150, po(2) + 15]);
                g = addGoto(sub, out, tagColor(cm, out));
                placeRight(sub, g, sub + "/" + sm, 1, 40);
                add_line(sub, p + "/1", sm + "/1");  add_line(sub, sm + "/1", g + "/1");
                blks = [blks, sm, g]; %#ok<AGROW>
            else
                % prodB into the sum from below: one corner
                q = get_param(sub + "/" + p, "PortHandles");  po = get_param(q.Outport, "Position");
                qs = get_param(sub + "/" + sm, "PortHandles");  pin = get_param(qs.Inport(2), "Position");
                add_line(sub, [po; pin(1), po(2); pin]);
            end
            y = y + 90;
        end
    end
    h = areaAround(sub, title, blks, areaColor("asset"));
    r = max(cell2mat(arrayfun(@(b) get_param(sub + "/" + b, "Position"), blks', "UniformOutput", false)) * [0; 0; 1; 0]);
    hO = ctrlOutputs(sub, outs, "@" + outs, r + 110, cm);
    alignColumns(hI, h, hO);
    % outputs level with their Goto
    for k = 1:numel(outs)
        gp = get_param(sub + "/" + outs(k) + " goto", "Position");  yc = (gp(2) + gp(4)) / 2;
        for b = [outs(k), outs(k) + " from 1"]
            bp = get_param(sub + "/" + b, "Position");  hh = bp(4) - bp(2);
            set_param(sub + "/" + b, "Position", [bp(1), yc - hh / 2, bp(3), yc + hh / 2]);
        end
    end
    set_param(sub, "BackgroundColor", areaColor("asset"), "Description", title);
end

function placeRight(sys, b, src, port, gap)
    q = get_param(src, "PortHandles");  po = get_param(q.Outport(port), "Position");
    p = get_param(sys + "/" + b, "Position");  w = p(3) - p(1);  h = p(4) - p(2);
    set_param(sys + "/" + b, "Position", round([po(1) + gap, po(2) - h / 2, po(1) + gap + w, po(2) + h / 2]));
end

function fillBlock(sub, ins, outs, rows, title, cm)
    % contents of a virtual subsystem of an asset: inports -> Goto, the rows
    % in one area, From -> outports
    [hI, x] = ctrlInputs(sub, ins, repmat("1", 1, numel(ins)), cm, "-1");
    [~, h, r] = ctrlSection(sub, title, rows, x, 20, cm, areaColor("asset"));
    hO = ctrlOutputs(sub, outs, "@" + outs, r + 110, cm);
    alignColumns(hI, h, hO);
    set_param(sub, "BackgroundColor", areaColor("asset"), "Description", title);
end

function contLag(sys, name, T)
    % continuous first-order lag 1/(T s + 1), initial output 0 (bypassed
    % with the averaged converter, see below), built as
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
    add_block("built-in/Outport", blk + "/y", "Position", [520 43 550 57]);
    % y = x (the lag) or u (bypass) by a Switch on the tunable
    % ap.conv.type: bypassed with the averaged converter (2), which models
    % the inner current loop explicitly; Tg is its RMS stand-in and counted
    % twice would add a 20 ms lag
    add_block("simulink/Sources/Constant", blk + "/converter model", "Value", "ap.conv.type", "Position", [280 95 350 115]);
    add_block("simulink/Signal Routing/Switch", blk + "/bypass?", "Criteria", "u2 > Threshold", "Threshold", "1.5", ...
        "Position", [430 30 460 120]);
    add_line(blk, "u/1", "err/1");  add_line(blk, "err/1", "gain/1");
    add_line(blk, "gain/1", "state/1");  add_line(blk, "state/1", "bypass?/3");
    add_line(blk, "u/1", "bypass?/1", "autorouting", "on");  add_line(blk, "converter model/1", "bypass?/2");
    add_line(blk, "bypass?/1", "y/1");
    add_line(blk, "state/1", "err/2", "autorouting", "on");
    set_param(blk, "Mask", "on", "MaskDisplay", "disp('1 / (Tg s + 1)')", "MaskIconOpaque", "off", ...
        "Description", "first-order lag Tg; bypassed with the averaged converter (ap.conv.type = 2)");
end
