function [k, caseName] = selectCase(cases, caseId, setName)
%SELECTCASE  Row of a case table from a case number or name.
%
%   [k, caseName] = selectCase(cases, caseId, setName): caseId is a number
%   (1..height) or a row name; empty is allowed for a set of one case.
%   caseName is the row name ("" for unnamed rows). Used by runCase and
%   loadResults.

    rowNames = string(cases.Properties.RowNames);
    n = height(cases);

    if isempty(caseId)
        if n ~= 1
            error("selectCase:noCase", ...
                  "Case set ""%s"" has %d cases. Give one, e.g. runCase(""%s"", 1).", ...
                  setName, n, setName);
        end
        k = 1;
    elseif isnumeric(caseId)
        if ~isscalar(caseId) || caseId ~= round(caseId) || caseId < 1 || caseId > n
            error("selectCase:badIndex", "Case set ""%s"" has cases 1-%d.", setName, n);
        end
        k = caseId;
    else
        k = find(rowNames == string(caseId), 1);
        if isempty(k)
            error("selectCase:badName", "Case set ""%s"" has no case named ""%s"". Names: %s", ...
                  setName, string(caseId), strjoin(rowNames, ", "));
        end
    end

    if isempty(rowNames)
        caseName = "";
    else
        caseName = rowNames(k);
    end
end
