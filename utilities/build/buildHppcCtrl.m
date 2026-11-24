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
    layoutHppc(mdl, outs, unused);
    ctrlSave(mdl, folder);
end

function layoutHppc(mdl, outs, unused)
    % layout in the style of the BESS_ctrl hand layout, as layoutRepc:
    % inputs | ramp / frequency response / export limit, P control, Q / V /
    % PF control, measurements and limits stacked | dispatch | outputs
    b = @(n) mdl + "/" + n;
    for n = ["Kpg Kig", "Q PI"], set_param(b(n), "Position", [0 0 70 150]); end
    for n = ["base + freq", "target + PI", "ff + Q PI"]
        set_param(b(n), "IconShape", "round", "Inputs", "|++", "Position", [0 0 30 30]);
    end
    set_param(b("V control?"), "Position", [0 0 40 160]);
    % ramp limit, frequency response, export limit
    putAt(mdl, "ramp limiter", 560, "in", 1, 70);
    placeLeft(mdl, "FSM reserve", b("ramp limiter"), 3, 20);  placeLeft(mdl, "Ts 2", b("ramp limiter"), 4, 20);
    yb = portY(b("ramp limiter"), "out", 1);
    putAt(mdl, "base + freq", 760, "in", 1, yb);
    putAt(mdl, "frequency response", 560, "in", 1, yb + 110);
    putAt(mdl, "export limit", 960, "in", 1, yb);  placeLeft(mdl, "Ppoc_max", b("export limit"), 2, 20);
    % P control at the POC
    yP = 400;
    putAt(mdl, "Tref", 330, "in", 1, yP);  putAt(mdl, "P error", 520, "in", 1, yP);
    putAt(mdl, "P loop on?", 680, "in", 1, portY(b("P error"), "out", 1));
    placeLeft(mdl, "Pflag", b("P loop on?"), 2, 15);  placeLeft(mdl, "no P loop", b("P loop on?"), 3, 15);
    putAt(mdl, "freeze in dip", 800, "in", 3, portY(b("P loop on?"), "out", 1));
    placeLeft(mdl, "frozen", b("freeze in dip"), 1, 15);
    yf = portY(b("freeze in dip"), "out", 1);
    putAt(mdl, "femax femin", 900, "in", 1, yf);  putAt(mdl, "Kpg Kig", 1080, "in", 1, yf);
    putAt(mdl, "Pmax - target", 1010, "out", 1, portY(b("Kpg Kig"), "in", 2));
    putAt(mdl, "Pmin - target", 1010, "out", 1, portY(b("Kpg Kig"), "in", 3));
    putAt(mdl, "target + PI", 1260, "in", 1, yf);
    putAt(mdl, "Pmax Pmin", 1420, "in", 2, portY(b("target + PI"), "out", 1));
    % Q, V or PF control: the V row above the main row
    yv = 650;
    putAt(mdl, "Kc", 340, "in", 1, yv);  putAt(mdl, "V + Kc Q", 440, "in", 2, yv);
    putAt(mdl, "V error", 620, "in", 2, portY(b("V + Kc Q"), "out", 1));  placeLeft(mdl, "Vref", b("V error"), 1, 15);
    putAt(mdl, "POC Q limit", 700, "in", 1, portY(b("V error"), "out", 1));
    putAt(mdl, "V control?", 900, "in", 1, portY(b("POC Q limit"), "out", 1));
    placeLeft(mdl, "RefFlag", b("V control?"), 2, 15);
    yQ = portY(b("V control?"), "in", 3);
    putAt(mdl, "Q error", 520, "out", 1, yQ);
    putAt(mdl, "TrefQ", 330, "out", 1, portY(b("Q error"), "in", 1));
    yo = portY(b("V control?"), "out", 1);
    putAt(mdl, "Q loop on?", 1080, "in", 1, yo);
    placeLeft(mdl, "Qflag", b("Q loop on?"), 2, 15);  placeLeft(mdl, "no Q loop", b("Q loop on?"), 3, 15);
    putAt(mdl, "freeze Q in dip", 1190, "in", 3, portY(b("Q loop on?"), "out", 1));
    placeLeft(mdl, "Q frozen", b("freeze Q in dip"), 1, 15);
    yq = portY(b("freeze Q in dip"), "out", 1);
    putAt(mdl, "eqmax eqmin", 1280, "in", 1, yq);
    putAt(mdl, "plant anti-windup", 1440, "out", 1, yq);   % its output level with the PI
    putAt(mdl, "Q PI", 1760, "in", 1, yq);
    putAt(mdl, "Qmax - ff", 1690, "out", 1, portY(b("Q PI"), "in", 2));  placeLeft(mdl, "Qmax", b("Qmax - ff"), 1, 15);
    putAt(mdl, "Qmin - ff", 1690, "out", 1, portY(b("Q PI"), "in", 3));  placeLeft(mdl, "Qmin", b("Qmin - ff"), 1, 15);
    putAt(mdl, "ff + Q PI", 1880, "in", 1, yq);
    putAt(mdl, "Qmax Qmin", 1950, "in", 1, yq);
    % Q reference rows below the main row: PF, feed-forward
    putAt(mdl, "tan phi", 340, "in", 1, yQ + 110);
    putAt(mdl, "PF control?", 520, "in", 1, yQ + 110);
    placeLeft(mdl, "PF mode", b("PF control?"), 2, 15);
    putAt(mdl, "Q feed-forward", 800, "in", 3, yQ + 140);
    placeLeft(mdl, "no Q ff", b("Q feed-forward"), 1, 15);  placeLeft(mdl, "RefFlag ff", b("Q feed-forward"), 2, 15);
    % measurements, SOC-aware BESS limits, P limits
    ym = yQ + 260;
    putAt(mdl, "P filt", 340, "in", 1, ym);  putAt(mdl, "Q filt", 340, "in", 1, ym + 50);
    putAt(mdl, "V filt", 340, "in", 1, ym + 100);
    putAt(mdl, "dip freeze", 620, "in", 1, ym + 40);   % beside Q filt, out of the P limit lines
    placeLeft(mdl, "Ts", b("dip freeze"), 2, 20);
    putAt(mdl, "SOC limits", 340, "in", 1, ym + 230);
    set_param(b("available"), "Position", [0 0 30 120]);
    putAt(mdl, "available", 800, "in", 3, portY(b("SOC limits"), "out", 1));
    putAt(mdl, "charge limit", 900, "in", 1, portY(b("SOC limits"), "out", 2) + 40);
    % dispatch, outputs
    putAt(mdl, "dispatch", 2100, "in", 1, portY(b("Pmax Pmin"), "out", 1));
    for k = 1:numel(outs), putAt(mdl, outs(k), 2400, "in", 1, portY(b("dispatch"), "out", k)); end
    % inputs level with the line they feed; unused ones with their terminator
    putAt(mdl, "P_ref", 40, "out", 1, portY(b("ramp limiter"), "in", 1));
    putAt(mdl, "f_meas", 40, "out", 1, portY(b("frequency response"), "in", 1));
    putAt(mdl, "Q_ref", 40, "out", 1, yQ + 180);
    putAt(mdl, "P_poc", 40, "out", 1, portY(b("P filt"), "in", 1));
    putAt(mdl, "Q_poc", 40, "out", 1, portY(b("Q filt"), "in", 1));
    putAt(mdl, "V_poc", 40, "out", 1, portY(b("V filt"), "in", 1));
    putAt(mdl, "P_dis_max_bess", 40, "out", 1, portY(b("SOC limits"), "in", 1));
    putAt(mdl, "P_chg_max_bess", 40, "out", 1, portY(b("SOC limits"), "in", 2));
    putAt(mdl, "SOC_bess", 40, "out", 1, portY(b("SOC limits"), "in", 3));
    putAt(mdl, "P_avail_wpp", 40, "out", 1, portY(b("available"), "in", 1));
    putAt(mdl, "P_avail_spp", 40, "out", 1, portY(b("available"), "in", 2));
    yu = portY(b("SOC limits"), "in", 3) + 80;
    for n = ["Q_sat_wpp", "Q_sat_spp", "Q_sat_bess"]
        putAt(mdl, n, 40, "out", 1, yu);  yu = yu + 40;
    end
    for n = unused
        putAt(mdl, n, 40, "out", 1, yu);  putAt(mdl, n + " unused", 120, "in", 1, yu);  yu = yu + 40;
    end
    % areas
    A = struct("title", {}, "pos", {}, "color", {});
    grp = {
        "Inputs", [string(get_param(find_system(mdl, "SearchDepth", 1, "BlockType", "Inport"), "Name"))', unused + " unused"], "interface"
        "Ramp limit, frequency response (LFSM-O, LFSM-U, FSM), export limit", ["ramp limiter", "FSM reserve", ...
            "Ts 2", "base + freq", "frequency response", "export limit", "Ppoc_max"], "controller"
        "P control at the POC (Pflag)", ["Tref", "P error", "P loop on?", "Pflag", "no P loop", "freeze in dip", ...
            "frozen", "femax femin", "Kpg Kig", "Pmax - target", "Pmin - target", "target + PI", "Pmax Pmin"], "controller"
        "Q, V or PF control at the POC (Qflag, RefFlag 0 1 2)", ["Kc", "V + Kc Q", "V error", "Vref", "POC Q limit", ...
            "V control?", "RefFlag", "TrefQ", "Q error", "Q loop on?", "Qflag", "no Q loop", "freeze Q in dip", ...
            "Q frozen", "eqmax eqmin", "plant anti-windup", "Q PI", "Qmax - ff", "Qmax", "Qmin - ff", "Qmin", ...
            "ff + Q PI", "Qmax Qmin", "tan phi", "PF control?", "PF mode", "Q feed-forward", "no Q ff", "RefFlag ff"], "controller"
        "Measurements, SOC-aware BESS limits, P limits", ["P filt", "Q filt", "V filt", "dip freeze", "Ts", ...
            "SOC limits", "available", "charge limit"], "interface"
        "Dispatch to the plants", "dispatch", "asset"
        "Outputs", outs, "interface"};
    xr = 2040;                                       % right edge of the function column
    for g = 1:size(grp, 1)
        n = grp{g, 2};
        P = cell2mat(arrayfun(@(x) get_param(b(x), "Position"), n(:), "UniformOutput", false));
        r = [min(P(:, 1)) - 20, min(P(:, 2)) - 40, max(P(:, 3)) + 20, max(P(:, 4)) + 30];
        if g > 1 && g < 6, r([1 3]) = [230, xr]; end
        if g == 6, r([1 3]) = [xr + 20, r(3) + 40]; end
        if g == 7, r(1) = 2350; end
        A(end+1) = struct("title", grp{g, 1}, "pos", r, "color", areaColor(grp{g, 3})); %#ok<AGROW>
    end
    t = ["export limit", "P_target";  "P filt", "P_filt";  "Q filt", "Q_filt";  "V filt", "V_filt"
         "dip freeze", "frz";  "available", "P_hi";  "charge limit", "P_lo"
         "SOC limits:1", "BESS_dis";  "SOC limits:2", "BESS_chg"
         "PF control?", "Q_refx";  "Q feed-forward", "Q_ff"
         "Q_ref", "Q_ref";  "Q_sat_wpp", "Q_sat_wpp";  "Q_sat_spp", "Q_sat_spp";  "Q_sat_bess", "Q_sat_bess"
         "P_avail_wpp", "P_avail_wpp";  "P_avail_spp", "P_avail_spp";  "V_poc", "V_poc"];
    keep = ["SOC limits->available", "SOC limits->charge limit", "P_avail_wpp->available", ...
            "P_avail_spp->available", "V_poc->V filt"];
    applyCodedLayout(mdl, A, dictionary(t(:, 1), t(:, 2)), keep);
end
