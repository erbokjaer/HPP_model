function cases = handpicked()
%HANDPICKED  Named, hand-picked cases. Row "nominal" is the project default.
%   NaN = keep the default from HPP_parameters. A row of all NaN is
%   therefore exactly the default parameter set.
%
%   Add a case: add a row name and one value per column.
%   Add a parameter: add a column (field path in p) with NaN for rows that
%   should keep the default.

    names     = ["nominal"; "lowRef"; "highRef"; "noBess"];
    grid_Pref = [NaN;       100e6;    200e6;     NaN    ];   % [W]
    bess_Pmax = [NaN;       NaN;      NaN;       0      ];   % [W]

    cases = table(grid_Pref, bess_Pmax, ...
        'VariableNames', ["grid.Pref", "bess.Pmax"], ...
        'RowNames',      names);
end
