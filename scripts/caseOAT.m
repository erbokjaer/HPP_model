function cases = caseOAT(varargin)
%CASEOAT  One-parameter-at-a-time case table.
%
%   cases = caseOAT("grid.Pref", [50 100 150 200]*1e6, "bess.Pmax", [0 15 30 60]*1e6)
%
%   Each row varies exactly one parameter; the other columns are NaN, which
%   applyCase treats as "keep the default from HPP_parameters".
%   Rows are named "<parameter>=<value>", e.g. "grid.Pref=1e+08".

    if isempty(varargin) || mod(numel(varargin), 2) ~= 0
        error("caseOAT:args", "Give name-value pairs: caseOAT(""grid.Pref"", [...], ...).");
    end
    names  = string(varargin(1:2:end));
    values = varargin(2:2:end);

    nRows = sum(cellfun(@numel, values));
    data  = NaN(nRows, numel(names));
    rowNames = strings(nRows, 1);

    r = 0;
    for k = 1:numel(names)
        for v = values{k}(:)'
            r = r + 1;
            data(r, k) = v;
            rowNames(r) = sprintf("%s=%g", names(k), v);
        end
    end

    cases = array2table(data, 'VariableNames', names, 'RowNames', rowNames);
end
