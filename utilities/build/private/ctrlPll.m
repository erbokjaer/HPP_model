%% ===================================================================
%  WT_ctrl template: IEC 61400-27-1 Type 4A control (stage 3). Structure
%  after IEC 61400-27-1 (measurement, P control type 4A, Q control,
%  current limitation); block details and parameter values to be checked
%  against the standard. Ports:
%    in  1 P_set  2 Q_set  3 V_term  4 P_meas  5 Q_meas  6 P_wind_avail
%    out 1 ip_cmd 2 iq_cmd 3 P_avail          (pu on the asset rating)
%  ===================================================================
function [h, r] = ctrlPll(mdl, x, y, cm)
    % PLL of an asset controller (stage 7): Model block of models/ctrl/PLL.slx
    % (buildPllModel), argument pll = par.pll (pllParameters, p.pll.asset).
    % Out: th_pll (the angle the converter injects along, rad) and f_pll
    % [pu] (the asset's measured frequency). In the controller, so a
    % converter DLL carries its own PLL, and the same controller serves the
    % RMS and the dq network.
    add_block("simulink/Ports & Subsystems/Model", mdl + "/PLL", "ModelName", "PLL", "Position", [0 0 120 70]);
    Simulink.ModelReference.refresh(mdl + "/PLL");
    setArg(mdl + "/PLL", "pll", "par.pll");
    rowsL = {[stg("@V_re", 0), stg("PLL", 1, {2, "@V_im"}, {2, "f_pll"}), stg(">th_pll")]};
    [~, h, r] = ctrlSection(mdl, "PLL (angle of the current injection, frequency)", rowsL, x, y, cm, areaColor("interface"));
end
