function cases = hppc_qv()
%HPPC_QV  Q and voltage control at the POC (stage 5).
%   Q control (default, RefFlag 0): TSO Q_ref steps at t = 40 s
%   (p.grid.Qstep); the open-loop HPPC for comparison (Q_poc keeps the
%   transformer and grid-side Q).
%   Voltage control (RefFlag 1): a sustained 2 % drop of the grid source
%   voltage from t = 40 s (p.grid.dip with a long duration); the HPPC
%   raises Q to hold V_poc at Vref.

    vdrop = {"grid.dip.depth", 0.02, "grid.dip.t0", 40, "grid.dip.dur", 100};
    vctrl = {"hppc.RefFlag", 1};
    cases = caseList({
        "Q 0"                     {}
        "Q step +30 Mvar"         {"grid.Qstep.dQ",  30e6}
        "Q step -30 Mvar"         {"grid.Qstep.dQ", -30e6}
        "Q open loop"             {"hppc.Qflag", 0}
        "V ctrl, grid -2 %"       [vctrl, vdrop]
        "V ctrl, grid -2 %, SCR3" [vctrl, vdrop, {"grid.SCR", 3}]
        "Q ctrl, grid -2 %"       vdrop
        "V ctrl, Vref 1.01"       [vctrl, {"hppc.Vref", 1.01}]
    });
end
