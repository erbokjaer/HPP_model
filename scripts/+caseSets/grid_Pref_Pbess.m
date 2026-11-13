function cases = grid_Pref_Pbess()
%GRID_PREF_PBESS  Full grid: every combination of Pref and BESS power rating.

    cases = caseGrid("grid.Pref", [100 150 200]*1e6, ...   % [W]
                     "bess.Pmax", [0 30 60]*1e6);          % [W]
end
