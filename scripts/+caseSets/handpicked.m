function cases = handpicked()
%HANDPICKED  Named, hand-picked cases. Row "nominal" is the project default.
%
%   One line per case: its name and only the parameters it changes, as
%   {"field.path", value, ...}. Everything not listed keeps the default
%   from HPP_parameters, so "nominal" lists nothing.
%
%   To add a case, add one line. The other cases never need editing.

    cases = caseList({
        "nominal",  {}
        "lowRef",   {"grid.Pref", 100e6}                  % [W]
        "highRef",  {"grid.Pref", 200e6}                  % [W]
        "noBess",   {"bess.Pmax", 0}                      % [W]
    });
end
