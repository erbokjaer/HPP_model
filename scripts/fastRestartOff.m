function wasOn = fastRestartOff(mdl)
%FASTRESTARTOFF  Switch Fast Restart off if the model is loaded with it on.
%
%   wasOn = fastRestartOff() for HPP.slx; fastRestartOff(mdl) for another
%   model. Needed before Rapid Accelerator (they exclude each other) and
%   when compile-time values changed.

    if nargin < 1, mdl = "HPP"; end
    wasOn = bdIsLoaded(mdl) && strcmp(get_param(mdl, "FastRestart"), "on");
    if wasOn
        set_param(mdl, "FastRestart", "off");
    end
end
