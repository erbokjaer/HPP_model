function cases = grid_strength()
%GRID_STRENGTH  Strong, medium and weak grid at the POC (short-circuit ratio).

    cases = caseList({
        "SCR10",  {}                      % default: strong grid
        "SCR5",   {"grid.SCR", 5}
        "SCR3",   {"grid.SCR", 3}         % weak grid
    });
end
