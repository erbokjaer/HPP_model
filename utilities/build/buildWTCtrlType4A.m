function buildWTCtrlType4A(folder)
    mdl = "WT_ctrl";
    ctrlHeader(mdl, "wt", ...
        "WT asset controller: IEC 61400-27-1 Type 4A control (measurement, P control type 4A, Q control " + ...
        "with closed-loop reactive power control (MqG) and dip current injection, current limitation). " + ...
        "Per unit on the asset rating. Parameters: model argument par (= p.ctrl.wt.par, from " + ...
        "p.wt.iec.ctrl). Structure after the standard; details and values to be checked against it.", false);
    cm.ref  = ["P_set", "Q_set", "P_wind_avail", "ip_req", "iq_req", "ip_cmd", "iq_cmd"];
    cm.meas = ["V_term", "P_meas", "Q_meas", "U", "U_floor", "Q_filt", "V_re", "V_im", "th_pll", "f_pll"];
    I = modelInterfaces();
    [hIn, x] = ctrlInputs(mdl, I.WT_ctrl.in, ...
        repmat("1", 1, 8), cm);

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
    [y, h4, r4] = ctrlSection(mdl, "Current limitation", rowsC, x, y, cm, areaColor("asset"));
    [h5, r5] = ctrlPll(mdl, x, y, cm);

    % outputs (Type 4A: constant aerodynamic power, P_avail = P_wind_avail)
    hOut = ctrlOutputs(mdl, I.(mdl).out, ...
        ["@ip_cmd", "@iq_cmd", "@P_wind_avail", "@th_pll", "@f_pll"], max([r1 r2 r3 r4 r5]) + 110, cm);
    alignColumns(hIn, [h1 h2 h3 h4 h5], hOut);
    % the template is the BESS_ctrl hand layout: its blocks by their role
    % here (block names, Goto tags and area titles)
    m = [ "pll_c", "th_pll"
          "SOC", "P_wind_avail"
          "Vt filt", "U filt";  "V floor", "U floor";  "Vdip", "udip";  "V < Vdip", "dip"
          "V_floor", "U_floor";  "P_meas unused", "P filt unused"
          "Pmax Pmin", "P_ref limit";  "Pmin", "P min";  "dPmax dPmin", "dpmaxp4A"
          "Tpord", "Tpordp4A";  "Pord to Ip", "p_ord to ip"
          "Qext to Iq", "q_ref to iq";  "ff + PI", "iq ff + PI";  "Q error", "q error"
          "Qflag", "MqG = 1";  "no Q loop", "no q loop";  "Q loop on?", "q loop on?"
          "Q frozen", "q frozen";  "Kqp Kqi", "kpq kiq";  "Vref0", "uref0";  "Vref0 - V", "uref0 - U"
          "Iqinj in dip", "in dip?";  "Iqh1 Iql1", "iq + dip"
          "current limit logic", "current limitation"
          "Voltage filter and dip detection", "Measurement and dip detection"
          "P path", "P control type 4A"
          "Q path (Qflag 1 closed loop, 0 open loop) and reactive current injection", ...
              "Q control (MqG 1 closed loop, 2 open loop) and dip current"
          "Current limit logic", "Current limitation"];
    % Type 4A differs from REEC: the dip current is added before the dip
    % switch (iq_ol into both, as Goto / From), iqH1 after it; Q and P are
    % filtered (in the measurement area, their inports level with them);
    % P_wind_avail feeds the P limit and P_avail (Goto / From, a line would
    % cross all inputs)
    place = {"iq + dip", 780, 238;  "iqH1", 960, 265
             "Q filt", 315, 560;  "P filt", 315, 620;  "P filt unused", 420, 620
             "Q_meas", NaN, 560;  "P_meas", NaN, 620};
    tags = dictionary(["iq ff + PI", "Q filt", "P_wind_avail", "dip"], ["iq_ol", "Q_filt", "P_wind_avail", "dip"]);
    applyCtrlTemplate(mdl, dictionary(m(:, 1), m(:, 2)), Place=place, Tags=tags);
    ctrlSave(mdl, folder);
end
