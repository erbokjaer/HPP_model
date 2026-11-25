function pllModelHeader(mdl, arg, src, desc)
    % new referenced model at the simulation step, model argument arg with its
    % default from HPP_parameters (p.<src>)
    closeIfLoaded(mdl);
    new_system(mdl);
    set_param(mdl, "SolverType", "Fixed-step", "Solver", "FixedStepDiscrete", "FixedStep", "Ts", ...
        "StopTime", "inf", "Description", desc);
    ws = get_param(mdl, "ModelWorkspace");
    ws.DataSource = "MATLAB Code";
    ws.MATLABCode = char(strjoin([
        "% Defaults for standalone use, from HPP_parameters. In HPP the Model"
        "% block passes the instance's parameters as the argument """ + arg + """."
        "pDefault = HPP_parameters();"
        "Ts = pDefault.sim.Ts;      % [s] simulation step"
        arg + " = pDefault." + src + ";"
        "clear pDefault"], newline));
    reload(ws);
    set_param(mdl, "ParameterArgumentNames", arg);
end
