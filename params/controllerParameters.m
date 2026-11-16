function ctrl = controllerParameters(p)
%CONTROLLERPARAMETERS  HPP plant controller parameters.
%
%   Takes the partially built p so controller limits can be derived from
%   component ratings instead of being duplicated.
%
%   The controller (HPP_controller.slx) works entirely in per unit on the
%   base ctrl.Pbase. Its interface:
%
%     Inputs   P_ref, P_wt_avail, P_pv_avail  [pu]    SOC  [-]
%     Outputs  P_wt_set, P_pv_set, P_bess_set [pu]    (+ = export / discharge)
%     Params   ctrl.par                       tunable, see below
%     Rate     ctrl.Ts                        fixed sample time
%
%   ctrl.par is the controller's only tunable parameter: a struct passed to
%   HPP_controller as a model argument. In generated code it becomes the
%   per-instance parameter struct, and later the parameter list of the
%   IEEE/CIGRE DLL. Put everything the controller needs in ctrl.par, in pu.
%
%   ctrl.Ts and ctrl.Pbase are fixed settings, not tunable: the sample
%   time cannot change at run time, and the base is only used by HPP.slx
%   to convert between W and pu around the controller.

    ctrl.Ts    = 0.1;            % [s]  controller sample time (fixed)
    ctrl.Pbase = p.grid.Pbase;   % [W]  power base of the pu interface (fixed)

    % Tunable parameters, all in pu on ctrl.Pbase
    ctrl.par.Pbess_max = p.bess.Pmax     / ctrl.Pbase;   % [pu] battery set-point limit
    ctrl.par.Ppoc_max  = p.grid.Pmax_poc / ctrl.Pbase;   % [pu] plant export limit

    assert(abs(ctrl.Ts/p.sim.Ts - round(ctrl.Ts/p.sim.Ts)) < 1e-9, ...
        "ctrl.Ts must be an integer multiple of sim.Ts");
end
