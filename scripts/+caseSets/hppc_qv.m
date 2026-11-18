function cases = hppc_qv()
%HPPC_QV  Q, voltage and power factor control at the POC (stage 5,
%   Energinet: RfG article 21(3)(d)).
%   Q control (default, RefFlag 0): TSO Q_ref steps at t = 40 s
%   (p.grid.Qstep); the open-loop HPPC for comparison.
%   Voltage control (RefFlag 1, slope Vslope 4 %, range 2-7 %): a sustained
%   2 % drop of the grid source voltage from t = 40 s (p.grid.dip with a
%   long duration). Requirement: 90 % of the Q change within t1 = 1 s,
%   settled within t2 = 5 s, tolerance 5 % of Qmax.
%   Power factor control (RefFlag 2): PF 0.95 at the POC.

    vdrop = {"grid.dip.depth", 0.02, "grid.dip.t0", 40, "grid.dip.dur", 100};
    vctrl = {"hppc.RefFlag", 1};
    cases = caseList({
        "Q 0"                       {}
        "Q step +30 Mvar"           {"grid.Qstep.dQ",  30e6}
        "Q step -30 Mvar"           {"grid.Qstep.dQ", -30e6}
        "Q open loop"               {"hppc.Qflag", 0}
        "V ctrl, grid -2 %"         [vctrl, vdrop]
        "V ctrl, grid -2 %, SCR3"   [vctrl, vdrop, {"grid.SCR", 3}]
        "V ctrl 2 %, grid -2 %"     [vctrl, vdrop, {"hppc.Vslope", 0.02}]
        "V ctrl 7 %, grid -2 %, SCR3" [vctrl, vdrop, {"hppc.Vslope", 0.07, "grid.SCR", 3}]
        "V ctrl, SCR3, set for SCR10" [vctrl, vdrop, {"grid.SCR", 3, "hppc.SCRset", 10}]
        "Q ctrl, grid -2 %"         vdrop
        "V ctrl, Vref 1.01"         [vctrl, {"hppc.Vref", 1.01}]
        "PF 0.95 export"            {"hppc.RefFlag", 2, "hppc.PF", 0.95}
        "PF 0.95 import"            {"hppc.RefFlag", 2, "hppc.PF", 0.95, "hppc.PFsign", -1}
    });
end
