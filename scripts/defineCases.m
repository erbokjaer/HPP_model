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
    % the cases of a set are plotted on one time axis: warn if they run for
    % different times (NaN = the default stop time)
    if ismember("sim.Tstop", cases.Properties.VariableNames)
        T = cases.("sim.Tstop");
        T(isnan(T)) = simulationParameters().Tstop;
        if numel(unique(T)) > 1
            warning("defineCases:mixedStopTime", "Case set ""%s"" mixes stop times (%s s); " + ...
                "its cases are plotted together, so give them all the same sim.Tstop.", ...
                setName, strjoin(string(unique(T)'), ", "));
        end
    end
end
