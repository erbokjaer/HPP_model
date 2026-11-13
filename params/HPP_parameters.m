function p = HPP_parameters(variant)
%HPP_PARAMETERS  Main parameter function for the hybrid power plant model.
%
%   p = HPP_parameters()           default plant
%   p = HPP_parameters('variant')  a named variant (see the switch below)
%
%   Returns one struct, p, which is the only workspace variable the
%   Simulink models use. Blocks reference fields, e.g. p.wt.Pn.
%
%   Each component lives in its own function so it can be reused, tested
%   and swapped independently. Override single values after loading,
%   e.g. for sweeps:   p = HPP_parameters();  p.ctrl.Kp = 0.8;

    arguments
        variant (1,1) string = "default"
    end

    p.sim  = simulationParameters();
    p.wt   = windTurbineParameters();
    p.pv   = pvParameters();
    p.bess = batteryParameters();
    p.grid = gridParameters();
    p.ctrl = controllerParameters(p);

    switch variant
        case "default"
            % nothing to change
        case "noBattery"
            p.bess.Pmax = 0;
        otherwise
            error("HPP_parameters:unknownVariant", ...
                  "Unknown variant '%s'.", variant);
    end

    p.meta.variant = variant;
    p.meta.created = string(datetime("now"));
end
