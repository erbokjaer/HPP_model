function cases = oat_Pref()
%OAT_PREF  One parameter at a time: vary Pref and BESS rating separately,
%   all other parameters at their defaults.

    cases = caseOAT("grid.Pref", [50 100 150 200]*1e6, ...   % [W]
                    "bess.Pmax", [0 15 30 60]*1e6);          % [W]
end
