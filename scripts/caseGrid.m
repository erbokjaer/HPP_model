function cases = caseGrid(varargin)
%CASEGRID  Full-grid case table: every combination of the given values.
%
%   cases = caseGrid("grid.Pref", [100 150 200]*1e6, "bess.Pmax", [0 30 60]*1e6)
%
%   Inputs are name-value pairs: a field path in p and a vector of values.
%   The first parameter varies fastest. Rows are unnamed (refer to them by
%   number).

    [names, values] = parsePairs(varargin);
    n = numel(names);

    grids = cell(1, n);
    if n == 1
        grids{1} = values{1}(:);
    else
        [grids{:}] = ndgrid(values{:});
    end

    cols = cellfun(@(g) g(:), grids, 'UniformOutput', false);
    cases = table(cols{:}, 'VariableNames', names);
end

function [names, values] = parsePairs(args)
    if isempty(args) || mod(numel(args), 2) ~= 0
        error("caseGrid:args", "Give name-value pairs: caseGrid(""grid.Pref"", [...], ...).");
    end
    names  = string(args(1:2:end));
    values = args(2:2:end);
end
