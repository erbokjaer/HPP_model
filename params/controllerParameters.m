function ctrl = controllerParameters(p)
%CONTROLLERPARAMETERS  HPP plant controller parameters.
%
%   Takes the partially built p so controller limits can be derived from
%   component ratings instead of being duplicated.
%
%   Only this struct is used inside the controller model (HPP_controller).
%   Keeping the controller's parameters in one place makes it easy later
%   to expose them as tunable parameters in generated code or as the
%   parameter list of an IEEE/CIGRE DLL.

    ctrl.Ts       = 0.1;              % [s]  controller sample time
    ctrl.Pbess_max = p.bess.Pmax;     % [W]  battery set-point limit
    ctrl.Ppoc_max  = p.grid.Pmax_poc; % [W]  plant export limit

    assert(abs(ctrl.Ts/p.sim.Ts - round(ctrl.Ts/p.sim.Ts)) < 1e-9, ...
        "ctrl.Ts must be an integer multiple of sim.Ts");
end
