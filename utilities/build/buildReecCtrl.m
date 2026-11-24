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
    cm.meas = ["V_term", "P_meas", "Q_meas", "V", "V_floor", "SOC", "V_re", "V_im", "th_pll", "f_pll"];
    I = modelInterfaces();
    [hIn, x] = ctrlInputs(mdl, I.(mdl).in, repmat("1", 1, numel(I.(mdl).in)), cm);

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
    [y, h4, r4] = ctrlSection(mdl, "Current limit logic", rowsC, x, y, cm, areaColor("asset"));
    [h5, r5] = ctrlPll(mdl, x, y, cm);

    % outputs: P_avail = available PV power, or the BESS converter limit
    if isB
        kc(mdl, "Pmax out", "par.Pmax");  pa = "Pmax out";
    else
        pa = "@P_pv_avail";
    end
    hOut = ctrlOutputs(mdl, I.(mdl).out, ...
        ["@ip_cmd", "@iq_cmd", pa, "@th_pll", "@f_pll"], max([r1 r2 r3 r4 r5]) + 110, cm);
    alignColumns(hIn, [h1 h2 h3 h4 h5], hOut);
    applyCtrlTemplate(mdl, dictionary(["pll_c", "SOC"], ["th_pll", src]));
    ctrlSave(mdl, folder);
end
