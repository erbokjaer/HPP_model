function labels = caseLabels(cases)
%CASELABELS  Short readable label for each row of a case table.
%
%   Named rows use their name ("noBess"). Unnamed rows list the values
%   they change, with powers in MW ("Pref=100 MW, Pmax=0 MW"). A row that
%   changes nothing is labelled "nominal".

    n = height(cases);
    labels = strings(n, 1);
    rowNames = string(cases.Properties.RowNames);
    if ~isempty(rowNames)
        labels = rowNames;
        return
    end

    vars = string(cases.Properties.VariableNames);
    for k = 1:n
        parts = strings(0, 1);
        for j = 1:numel(vars)
            v = cases{k, j};
            if isnumeric(v) && isscalar(v) && isnan(v)
                continue
            end
            parts(end+1) = formatValue(vars(j), v); %#ok<AGROW>
        end
        if isempty(parts)
            labels(k) = "nominal";
        else
            labels(k) = strjoin(parts, ", ");
        end
    end
end

function s = formatValue(path, v)
    name = extractAfter(path, ".");
    if name == ""
        name = path;
    end
    if startsWith(name, "P") && isnumeric(v)
        s = sprintf("%s=%g MW", name, v / 1e6);
    else
        s = sprintf("%s=%g", name, v);
    end
end
