function I = modelInterfaces()
%MODELINTERFACES  Ports and parameters that HPP.slx expects of each controller and asset model.
%
%   I = modelInterfaces()   struct with one field per model (HPPC, WPPC,
%                           SPPC, BESSC, WT_ctrl, PV_ctrl, BESS_ctrl,
%                           WT_asset, PV_asset, BESS_asset), each with
%       folder   "ctrl" or "assets" (under models/)
%       in, out  port names in port order (HPP.slx connects by port number)
%       arg      model argument name: "par" (controllers) or "ap" (assets)
%       src      where the argument's default comes from in HPP_parameters():
%                a field path of p, e.g. "ctrl.hppc.par" or "wt.ap(1)"
%
%   The one place these lists are defined: buildHPP uses them for the HPPC
%   template and its wiring (buildHppcCtrl, buildHppcSub), and checkModels
%   compares the hand-maintained models against them. When a model's ports
%   change on purpose, change them here and in buildHPP's wiring together.

    I = struct();

    % HPP controller: TSO, POC, then per plant its measurements with Q_sat last
    I.HPPC = iface("ctrl", ...
        ["P_ref", "Q_ref", "f_meas", "P_poc", "Q_poc", "V_poc", ...
         "P_avail_wpp", "P_wpp", "Q_wpp", "Q_sat_wpp", ...
         "P_avail_spp", "P_spp", "Q_spp", "Q_sat_spp", ...
         "P_chg_max_bess", "P_dis_max_bess", "SOC_bess", "P_bess", "Q_bess", "Q_sat_bess"], ...
        ["P_set_wpp", "Q_set_wpp", "P_set_spp", "Q_set_spp", "P_set_bess", "Q_set_bess"], ...
        "par", "ctrl.hppc.par");

    % plant controllers (WECC REPC_A); asset vectors of width N
    plantIn  = ["P_set", "Q_set", "P_meas", "Q_meas", "V_meas", "P_avail_k"];
    plantOut = ["P_set_k", "Q_set_k", "P_avail", "P_plant", "Q_plant"];
    I.WPPC  = iface("ctrl", [plantIn, "f_meas"], [plantOut, "Q_sat"], "par", "ctrl.wppc.par");
    I.SPPC  = iface("ctrl", [plantIn, "f_meas"], [plantOut, "Q_sat"], "par", "ctrl.sppc.par");
    I.BESSC = iface("ctrl", [plantIn, "SOC_k", "f_meas"], ...
        [plantOut, "P_chg_max", "P_dis_max", "SOC", "Q_sat"], "par", "ctrl.bessc.par");

    % asset controllers (inside the asset models); the terminal voltage
    % phasor (V_re, V_im) feeds their PLL, which gives the angle of the
    % current injection (th_pll, rad) and the asset frequency f_pll
    assetCtrlIn = ["P_set", "Q_set", "V_term", "P_meas", "Q_meas"];
    assetCtrlOut = ["ip_cmd", "iq_cmd", "P_avail", "th_pll", "f_pll"];
    pllIn = ["V_re", "V_im"];
    I.WT_ctrl   = iface("ctrl", [assetCtrlIn, "P_wind_avail", pllIn], assetCtrlOut, "par", "ctrl.wt.par");
    I.PV_ctrl   = iface("ctrl", [assetCtrlIn, "P_pv_avail", pllIn],   assetCtrlOut, "par", "ctrl.pv.par");
    I.BESS_ctrl = iface("ctrl", [assetCtrlIn, "SOC", pllIn],          assetCtrlOut, "par", "ctrl.bess.par");

    % PLL (stage 7, in the asset controllers and FreqMeter) and the
    % controllers' frequency meter (in the plant and HPP subsystems)
    I.PLL       = iface("ctrl", ["V_re", "V_im"], ["th", "f"], "pll", "pll.asset");
    I.FreqMeter = iface("ctrl", ["V_re", "V_im", "f_sys"], ["f", "RoCoF"], "m", "pll.hppc");

    % asset models (plant side)
    assetIn = ["P_set", "Q_set", "V_re", "V_im"];
    assetOut = ["P", "Q", "P_avail", "I_re", "I_im"];
    I.WT_asset   = iface("assets", [assetIn, "P_wind_avail"], assetOut, "ap", "wt.ap(1)");
    I.PV_asset   = iface("assets", [assetIn, "P_pv_avail"],   assetOut, "ap", "pv.ap(1)");
    I.BESS_asset = iface("assets", assetIn, [assetOut, "SOC"], "ap", "bess.ap(1)");
end

function s = iface(folder, in, out, arg, src)
    s = struct("folder", folder, "in", in, "out", out, "arg", arg, "src", src);
end
