function cases = defineCases(setName)
%DEFINECASES  Case table for a named case set (default: "nominal").
%
%   cases = defineCases()                    nominal case
%   cases = defineCases("grid_Pref_Pbess")   any set in scripts/+caseSets
%   listCaseSets()                           show the available sets
%
%   One row per case. Each column name is a field path in p, e.g.
%   "grid.Pref" sets p.grid.Pref. NaN means "keep the default".
%   To add a case set, add a function to scripts/+caseSets that returns
%   such a table (see caseGrid, caseOAT and caseSets.handpicked).

    arguments
        setName (1,1) string = "nominal"
    end

    available = listCaseSets();
    if ~ismember(setName, available)
        error("defineCases:unknownSet", ...
              "Unknown case set ""%s"". Available: %s", ...
              setName, strjoin(available, ", "));
    end

    cases = feval("caseSets." + setName);

    if ~istable(cases) || height(cases) == 0
        error("defineCases:empty", "Case set ""%s"" must return a non-empty table.", setName);
    end
end
