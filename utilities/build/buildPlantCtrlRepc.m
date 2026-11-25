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
        [stg("@Q_ff", 0), stg("ff + PI", 1, {2, "@Q_PI"}), stg(">Q_u")]
        [stg("@Q_u", 0), stg("Tfv"), stg(">Q_ord")]
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
    layoutRepc(mdl, isB, up, outs);
    ctrlSave(mdl, folder);
end

function layoutRepc(mdl, isB, up, outs)
    % layout in the style of the BESS_ctrl hand layout (docs/layout_rules.md):
    % inputs | frequency response, P control, Q or V control, measurements
    % stacked | dispatch | outputs; each path on rows with its ports level,
    % PI blocks stretched so their limits come in straight, round sums fed
    % from below; signals used in several places as labels, lines elsewhere
    b = @(n) mdl + "/" + n;
    set_param(b("Kpg Kig"), "Position", [0 0 70 150]);
    set_param(b("Kp Ki"), "Position", [0 0 70 150]);
    set_param(b("P_set + droop"), "Position", [0 0 30 120]);   % rows 40 apart
    set_param(b("V control?"), "Position", [0 0 40 160]);      % V row well above the main row
    set_param(b("ref + PI"), "IconShape", "round", "Inputs", "|++", "Position", [0 0 30 30]);
    set_param(b("ff + PI"), "IconShape", "round", "Inputs", "|++", "Position", [0 0 30 30]);
    % frequency response
    putAt(mdl, "P_set + droop", 660, "in", 1, 53);
    yU = portY(b("P_set + droop"), "in", 2);  yD = portY(b("P_set + droop"), "in", 3);
    putAt(mdl, "Dup", 570, "out", 1, yU);  putAt(mdl, "under-f", 490, "out", 1, yU);
    putAt(mdl, "Ddn", 570, "out", 1, yD);  putAt(mdl, "over-f", 490, "out", 1, yD);
    putAt(mdl, "fdbd", 410, "out", 1, (yU + yD) / 2);  putAt(mdl, "f error", 330, "out", 1, (yU + yD) / 2);
    placeLeft(mdl, "Freq_ref", b("f error"), 1, 25);
    % P control
    yP = 260;
    putAt(mdl, "femax femin", 400, "out", 1, yP);  putAt(mdl, "P error", 320, "out", 1, yP);
    putAt(mdl, "Kpg Kig", 620, "in", 1, yP);
    putAt(mdl, "Pmax - ref", 540, "out", 1, portY(b("Kpg Kig"), "in", 2));
    putAt(mdl, "Pmin - ref", 540, "out", 1, portY(b("Kpg Kig"), "in", 3));
    putAt(mdl, "ref + PI", 740, "in", 1, yP);
    putAt(mdl, "closed loop?", 930, "in", 1, yP);  placeLeft(mdl, "Frqflag", b("closed loop?"), 2, 20);
    putAt(mdl, "Pmax Pmin", 1110, "in", 2, portY(b("closed loop?"), "out", 1));
    putAt(mdl, "Tlag", 1210, "in", 1, portY(b("Pmax Pmin"), "out", 1));
    % Q or V control: the V row above the main row
    yv = 540;
    putAt(mdl, "Q - Q_set", 320, "out", 1, yv);  putAt(mdl, "Kc", 380, "in", 1, yv);
    putAt(mdl, "V + Kc Q", 470, "in", 2, yv);
    putAt(mdl, "V error", 640, "in", 2, portY(b("V + Kc Q"), "out", 1));  placeLeft(mdl, "Vref", b("V error"), 1, 15);
    putAt(mdl, "V control?", 800, "in", 1, portY(b("V error"), "out", 1));
    placeLeft(mdl, "RefFlag", b("V control?"), 2, 15);
    yQ = portY(b("V control?"), "in", 3);
    putAt(mdl, "Q error", 320, "out", 1, yQ);
    yo = portY(b("V control?"), "out", 1);
    putAt(mdl, "dbd", 880, "in", 1, yo);  putAt(mdl, "emax emin", 950, "in", 1, yo);
    putAt(mdl, "freeze", 1030, "in", 3, yo);  placeLeft(mdl, "frozen", b("freeze"), 1, 15);
    yf = portY(b("freeze"), "out", 1);
    putAt(mdl, "Kp Ki", 1280, "in", 1, yf);
    putAt(mdl, "Qmax - ff", 1210, "out", 1, portY(b("Kp Ki"), "in", 2));  placeLeft(mdl, "Qmax", b("Qmax - ff"), 1, 15);
    putAt(mdl, "Qmin - ff", 1210, "out", 1, portY(b("Kp Ki"), "in", 3));  placeLeft(mdl, "Qmin", b("Qmin - ff"), 1, 15);
    putAt(mdl, "ff + PI", 1390, "in", 1, yf);
    putAt(mdl, "Tfv", 1450, "in", 1, yf);
    putAt(mdl, "Q saturation", 1450, "in", 1, yf + 60);
    putAt(mdl, "Q feed-forward", 470, "in", 3, yQ + 90);
    placeLeft(mdl, "no ff", b("Q feed-forward"), 1, 15);  placeLeft(mdl, "RefFlag ff", b("Q feed-forward"), 2, 15);
    putAt(mdl, "V < Vfrz", 680, "out", 1, yQ + 120);  placeLeft(mdl, "Vfrz", b("V < Vfrz"), 2, 15);
    % measurements and P limits
    q = get_param(b("Kp Ki"), "Position");  ym = max(q(4), yQ + 150) + 90;
    putAt(mdl, "P filt", 330, "in", 1, ym);  putAt(mdl, "Q filt", 330, "in", 1, ym + 50);
    putAt(mdl, "V filt", 330, "in", 1, ym + 100);  putAt(mdl, "P_avail sum", 330, "in", 1, ym + 150);
    if isB
        putAt(mdl, "BESS limits", 480, "in", 1, ym + 150);
        putAt(mdl, "charge limit", 800, "in", 1, ym + 150);   % right of the [P_chg_max] label
    else
        putAt(mdl, "Pmin", 480, "out", 1, ym + 200);
    end
    % dispatch and outputs
    putAt(mdl, "dispatch", 1680, "in", 1, portY(b("Tlag"), "out", 1));
    putAt(mdl, "P_set_k", 1960, "in", 1, portY(b("dispatch"), "out", 1));
    putAt(mdl, "Q_set_k", 1960, "in", 1, portY(b("dispatch"), "out", 2));
    yk = portY(b("dispatch"), "out", 2);
    for k = 3:numel(outs)
        putAt(mdl, outs(k), 1960, "in", 1, yk + 60 * (k - 2));
    end
    % inputs level with the line they feed
    putAt(mdl, "P_set", 40, "out", 1, portY(b("P_set + droop"), "in", 1));
    putAt(mdl, "f_meas", 40, "out", 1, portY(b("f error"), "in", 2));
    putAt(mdl, "Q_set", 40, "out", 1, portY(b("Q error"), "in", 1));
    putAt(mdl, "P_meas", 40, "out", 1, portY(b("P filt"), "in", 1));
    putAt(mdl, "Q_meas", 40, "out", 1, portY(b("Q filt"), "in", 1));
    putAt(mdl, "V_meas", 40, "out", 1, portY(b("V filt"), "in", 1));
    putAt(mdl, "P_avail_k", 40, "out", 1, portY(b("P_avail sum"), "in", 1));
    if isB, putAt(mdl, "SOC_k", 40, "out", 1, portY(b("BESS limits"), "in", 2)); end
    % areas around their blocks (room for the labels in front)
    A = struct("title", {}, "pos", {}, "color", {});
    grp = {
        "Inputs", ["P_set", "f_meas", "Q_set", "P_meas", "Q_meas", "V_meas", "P_avail_k", "SOC_k"], "interface"
        "Frequency response (Ddn, Dup)", ["f error", "fdbd", "under-f", "over-f", "Dup", "Ddn", "P_set + droop", "Freq_ref"], "controller"
        "P control (Frqflag)", ["P error", "femax femin", "Kpg Kig", "Pmax - ref", "Pmin - ref", "ref + PI", ...
            "closed loop?", "Frqflag", "Pmax Pmin", "Tlag"], "controller"
        "Q or V control (RefFlag)", ["Q - Q_set", "Kc", "V + Kc Q", "V error", "Vref", "V control?", "RefFlag", ...
            "Q error", "dbd", "emax emin", "freeze", "frozen", "Kp Ki", "Qmax - ff", "Qmax", "Qmin - ff", "Qmin", ...
            "ff + PI", "Tfv", "Q saturation", "Q feed-forward", "no ff", "RefFlag ff", "V < Vfrz", "Vfrz"], "controller"
        "Measurements and P limits", ["P filt", "Q filt", "V filt", "P_avail sum", "Pmin", "BESS limits", "charge limit"], "interface"
        "Dispatch to the assets", "dispatch", "asset"
        "Outputs", outs, "interface"};
    for g = 1:size(grp, 1)
        n = grp{g, 2};  n = n(arrayfun(@(x) getSimulinkBlockHandle(b(x)) ~= -1, n));
        P = cell2mat(arrayfun(@(x) get_param(b(x), "Position"), n(:), "UniformOutput", false));
        r = [min(P(:, 1)) - 20, min(P(:, 2)) - 40, max(P(:, 3)) + 20, max(P(:, 4)) + 30];
        if g > 1 && g < size(grp, 1), r(1) = 230; end            % one column of function areas
        if any(grp{g, 1} == ["Frequency response (Ddn, Dup)", "P control (Frqflag)", ...
                "Q or V control (RefFlag)", "Measurements and P limits"])
            r(3) = 1610;
        end
        if grp{g, 1} == "Dispatch to the assets", r([1 3]) = [1630, r(3) + 40]; end
        if grp{g, 1} == "Outputs", r(1) = 1900; end
        A(end+1) = struct("title", grp{g, 1}, "pos", r, "color", areaColor(grp{g, 3})); %#ok<AGROW>
    end
    % labels: signals used in several places; the first use of an input
    % stays a line (Keep)
    t = ["P_set + droop", "P_ref";  "P filt", "P_filt";  "Q filt", "Q_filt";  "V filt", "V_filt"
         "P_avail sum", "P_avail";  "Q feed-forward", "Q_ff";  "V < Vfrz", "V_low";  "Q saturation", "Q_sat"
         "P_set", "P_set";  "Q_set", "Q_set";  "P_meas", "P_meas";  "Q_meas", "Q_meas";  "P_avail_k", "P_avail_k"];
    keep = ["P_set->P_set + droop", "Q_set->Q error", "P_meas->P filt", "Q_meas->Q filt", "P_avail_k->P_avail sum"];
    if isB
        t = [t; "BESS limits:1", "P_chg_max";  "BESS limits:2", "P_dis_max";  "BESS limits:3", "SOC";  "charge limit", "P_lo";  "SOC_k", "SOC_k"];
        keep = [keep, "SOC_k->BESS limits", "P_avail_k->BESS limits", "BESS limits->charge limit"];
    else
        t = [t; "Pmin", "P_lo"];
    end
    applyCodedLayout(mdl, A, dictionary(t(:, 1), t(:, 2)), keep);
end
