function cases = caseList(list)
%CASELIST  Case table from a list of named cases, one case per row.
%
%   cases = caseList({
%       "nominal",  {}
%       "lowRef",   {"grid.Pref", 100e6}
%       "noBess",   {"bess.Pmax", 0}
%       "small",    {"grid.Pref", 100e6, "bess.Pmax", 15e6}
%   });
%
%   Each row of the N-by-2 cell array is a case name and a cell array of
%   name-value pairs (field path in p, value). A case only lists the
%   parameters it changes: every other column is NaN, which applyCase
%   treats as "keep the default from HPP_parameters". Adding a case or a
%   parameter therefore never requires editing the other cases.
%
%   Columns appear in the order the parameters are first used.

    if ~iscell(list) || size(list, 2) ~= 2
        error("caseList:shape", "Give an N-by-2 cell array: {name, {path, value, ...}; ...}.");
    end

    names = string(list(:, 1));
    if any(names == "") || numel(unique(names)) ~= numel(names)
        error("caseList:names", "Case names must be non-empty and unique.");
    end

    % Collect the parameter columns in order of first use
    params = strings(0, 1);
    for k = 1:numel(names)
        pairs = list{k, 2};
        if ~iscell(pairs) || mod(numel(pairs), 2) ~= 0
            error("caseList:pairs", "Case ""%s"": give parameters as {""path"", value, ...}.", names(k));
        end
        params = [params; string(pairs(1:2:end)).']; %#ok<AGROW>
    end
    params = unique(params, "stable");

    % Fill the table; NaN = keep default
    data = NaN(numel(names), numel(params));
    for k = 1:numel(names)
        pairs = list{k, 2};
        for j = 1:2:numel(pairs)
            v = pairs{j + 1};
            if ~(isnumeric(v) || islogical(v)) || ~isscalar(v)
                error("caseList:value", "Case ""%s"", ""%s"": values must be numeric scalars.", ...
                      names(k), string(pairs{j}));
            end
            data(k, params == string(pairs{j})) = v;
        end
    end

    cases = array2table(data, 'VariableNames', params, 'RowNames', names);
end
